// yack: planar faces moved onto new planes, every vertex where its planes meet (from lib.rs, hardening T1).
// Adapted from springrep@a03e8a1 src/edit/tweak.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from edit/tweak.rs to tweak/mod.rs (M2.4, T6) and to yack-tweak (M3 W7); split out of
// lib.rs by what it does (hardening T1); checked access, the solid finished by the crate's `finished`
// (hardening T2); its refusals checked in face-id order (hardening T3).
//! Planar faces moved onto new planes with the solid's topology kept: each vertex goes where its faces'
//! planes meet, and each straight edge becomes the line between its moved ends.

#[allow(clippy::disallowed_types)] use std::collections::HashMap; // D26: BTree* in hardening, proven by digests

use crate::{
    Error, Result, clear_of_contacts, edge_faces, entry, finished,
    geom::{Curve, Line, Plane, Surface},
    math::{Frame, Point3, Vec2, Vec3},
    numeric::solve_dense,
    plane_of,
    surfaces::move_faces,
    tol,
    topo::{Brep, CoedgeSpec, FaceId, PCurve},
    unsupported, vertex_faces,
};

/// The point on every plane `(n, d)`, `n·p = d`, by least squares, when they pin one point.
fn meet(planes: &[(Vec3, f64)]) -> Option<Point3> {
    // The normal equations: a = Σ n nᵀ, r = Σ n d.
    let mut a = [[0.0; 3]; 3];
    let mut r = [0.0; 3];
    for &(n, d) in planes {
        let n = n.to_array();
        for (row, &ni) in a.iter_mut().zip(&n) {
            for (cell, &nj) in row.iter_mut().zip(&n) {
                *cell += ni * nj;
            }
        }
        for (ri, &ni) in r.iter_mut().zip(&n) {
            *ri += ni * d;
        }
    }
    let [[a0, a1, a2], [a3, a4, a5], [a6, a7, a8]] = a;
    let det = a0 * (a4 * a8 - a5 * a7) - a1 * (a3 * a8 - a5 * a6) + a2 * (a3 * a7 - a4 * a6);
    // Unit normals: a determinant this small leaves the point free along a line, to rounding.
    if det.abs() < 1e-12 {
        return None;
    }
    let &[x, y, z] = solve_dense(a.iter().flatten().copied().collect(), r.to_vec(), 3, 1).ok()?.as_slice() else {
        return None;
    };
    let p = Vec3::new(x, y, z);
    planes
        .iter()
        .all(|&(n, d)| (n.dot(p) - d).abs() <= tol::LINEAR)
        .then_some(p)
}

/// The line's parameter line in the plane of `frame`.
fn affine_on(frame: &Frame, line: &Line) -> PCurve {
    let l = frame.to_local(line.origin);
    PCurve::Affine {
        origin: Vec2::new(l.x, l.y),
        dir: Vec2::new(line.dir.dot(frame.x), line.dir.dot(frame.y)),
    }
}

/// [`move_planes`], or [`move_faces`] where a curved face meets the moved planes.
#[allow(clippy::disallowed_types)] pub(crate) fn planes_or_faces(b: &Brep, moved: &HashMap<FaceId, Frame>) -> Result<Brep> { // D26: BTree* in hardening, proven by digests
    match move_planes(b, moved) {
        Err(Error::Unsupported(_)) => {
            let surfaces = moved
                .iter()
                .map(|(&f, &frame)| (f, Surface::Plane(Plane { frame })))
                .collect();
            move_faces(b, &surfaces)
        }
        r => r,
    }
}

/// `b` with each face of `moved` on its new plane, given by a frame whose `z` is the face's new
/// natural normal (on the side of its old one).
#[allow(clippy::disallowed_types)] pub(crate) fn move_planes(b: &Brep, moved: &HashMap<FaceId, Frame>) -> Result<Brep> { // D26: BTree* in hardening, proven by digests
    clear_of_contacts(b, moved.keys())?;
    // In face-id order, so that of two faces refused two ways the same one speaks every run.
    let mut by_id: Vec<(&FaceId, &Frame)> = moved.iter().collect();
    by_id.sort_unstable_by_key(|&(f, _)| *f);
    for (&f, fr) in by_id {
        let Some(old) = plane_of(&b.face(f).surface) else {
            return unsupported("moving a face that is not a plane");
        };
        if old.z.dot(fr.z) <= 0.0 {
            return Err(Error::InvalidInput(
                "a moved plane must face the way the old one did".into(),
            ));
        }
    }
    let frame_of = |f: FaceId| moved.get(&f).copied().or(plane_of(&b.face(f).surface));
    let around = vertex_faces(b);
    let mut out = Brep::new();
    // By `b`'s vertex, in id order: the new vertex and whether it moved.
    let mut verts = Vec::new();
    let mut shifted = Vec::new();
    for v in b.vertex_ids() {
        let faces = around.get(&v).cloned().unwrap_or_default();
        let p = b.vertex(v).point;
        if !faces.iter().any(|f| moved.contains_key(f)) {
            verts.push(out.add_vertex(p));
            shifted.push(false);
            continue;
        }
        let mut planes = Vec::new();
        for &f in &faces {
            let Some(fr) = frame_of(f) else {
                return unsupported("moving a corner where a curved face meets the planes");
            };
            planes.push((fr.z, fr.z.dot(fr.origin)));
        }
        let Some(q) = meet(&planes) else {
            return unsupported("moving a corner whose planes do not meet at one point");
        };
        shifted.push(q.dist(p) > 0.0);
        verts.push(out.add_vertex(q));
    }
    let edge_faces = edge_faces(b);
    // By `b`'s edge, in id order: the new edge and whether it was laid anew.
    let mut edges = Vec::new();
    let mut rebuilt = Vec::new();
    for e in b.edge_ids() {
        let x = b.edge(e);
        let (s, t) = (entry(&verts, x.start.index())?, entry(&verts, x.end.index())?);
        let touched = edge_faces
            .get(&e)
            .is_some_and(|fs| fs.iter().any(|f| moved.contains_key(f)))
            || entry(&shifted, x.start.index())?
            || entry(&shifted, x.end.index())?;
        let (id, laid) = match &x.curve {
            Some(Curve::Line(old)) if touched => {
                let (a, z) = (out.vertex(s).point, out.vertex(t).point);
                let Some((line, len)) = Line::through(a, z) else {
                    return Err(Error::InvalidResult(vec![
                        "a moved edge shrinks to nothing".into(),
                    ]));
                };
                // A face turned inside out both ways keeps its loops' sense, so the checker
                // passes it; its edges are what run backwards.
                if line.dir.dot(old.dir) <= 0.0 {
                    return Err(Error::InvalidResult(vec![
                        "a moved edge runs backwards: a face turns inside out".into(),
                    ]));
                }
                (out.add_edge(Curve::Line(line), (0.0, len), s, t), true)
            }
            _ if touched => {
                return unsupported("moving a face bounded by a curved edge");
            }
            Some(c) => (out.add_edge(c.clone(), x.range, s, t), false),
            None => (out.add_degenerate_edge(s, x.range), false),
        };
        edges.push(id);
        rebuilt.push(laid);
    }
    let mut faces = Vec::new();
    for f in b.face_ids() {
        let face = b.face(f);
        let surface = match moved.get(&f) {
            Some(&frame) => Surface::Plane(Plane { frame }),
            None => face.surface.clone(),
        };
        let plane = plane_of(&surface);
        let nf = out.add_face(surface, face.reversed, face.names.clone());
        for &lp in &face.loops {
            let specs = b
                .lp(lp)
                .coedges
                .iter()
                .map(|&c| {
                    let co = b.coedge(c);
                    let edge = entry(&edges, co.edge.index())?;
                    let pcurve = match (&out.edge(edge).curve, plane) {
                        (Some(Curve::Line(l)), Some(fr))
                            if entry(&rebuilt, co.edge.index())? || moved.contains_key(&f) =>
                        {
                            affine_on(&fr, l)
                        }
                        _ => co.pcurve.clone(),
                    };
                    Ok(CoedgeSpec {
                        edge,
                        reversed: co.reversed,
                        pcurve,
                    })
                })
                .collect::<Result<Vec<_>>>()?;
            out.add_loop(nf, specs);
        }
        faces.push(nf);
    }
    finished(b, out, &faces)
}
