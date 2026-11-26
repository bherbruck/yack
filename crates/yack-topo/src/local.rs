// yack: a solid's shape at one edge or face (moved from springrep's facade/api.rs, M3 W10).
//! A solid's shape at one place: an edge's convexity and points, a face's outward normal, and the
//! local checks the facade's edits make before they run.

use crate::{Brep, EdgeId, FaceId};
use crate::math::{Point3, Vec3};

/// How the two faces meet along `e`, at its midpoint: `Some(true)` convex (material in a wedge
/// narrower than a half turn), `Some(false)` concave, `None` where they meet tangentially, or
/// the edge is not used by exactly two faces.
pub fn convexity(b: &Brep, e: EdgeId) -> Option<bool> {
    let edge = b.edge(e);
    let c = edge.curve.as_ref()?;
    let [cf, cg] = b.edge_uses(e) else { return None };
    let (f, g) = (b.coedge_face(*cf), b.coedge_face(*cg));
    if f == g {
        return None;
    }
    let outward = |f: FaceId, p: Point3| {
        let s = &b.face(f).surface;
        let uv = s.project(p);
        let n = s.normal(uv.x, uv.y);
        if b.face(f).reversed { -n } else { n }
    };
    let (p, dp) = c.d1(0.5 * (edge.range.0 + edge.range.1));
    let (n, m) = (outward(f, p), outward(g, p));
    if n.cross(m).norm() <= crate::tol::TANGENT {
        return None;
    }
    let t = if b.coedge(*cf).reversed { -dp } else { dp };
    Some(m.dot(n.cross(t)) < 0.0)
}

/// The outward unit normal of face `f`'s own surface at the point of it nearest `p`.
pub fn face_normal_at(b: &Brep, f: FaceId, p: Point3) -> Option<Vec3> {
    let face = b.face(f);
    let uv = face.surface.project(p);
    let n = face.surface.normal(uv.x, uv.y).unit()?;
    Some(if face.reversed { -n } else { n })
}

/// `n` points evenly spaced in parameter along edge `e`, ends included (empty for a pole).
pub fn edge_points(b: &Brep, e: EdgeId, n: usize) -> Vec<Point3> {
    let edge = b.edge(e);
    let Some(c) = &edge.curve else { return Vec::new() };
    let (t0, t1) = edge.range;
    let pts: Vec<Point3> = (0..n).map(|i| c.eval(if n <= 1 { t0 } else { t0 + (t1 - t0) * i as f64 / (n - 1) as f64 })).collect();
    pts
}

/// Edge `e`'s point at curve parameter `t`, and the edge's parameter range.
pub fn edge_point(b: &Brep, e: EdgeId, t: f64) -> Option<Point3> {
    b.edge(e).curve.as_ref().map(|c| c.eval(t))
}

/// Whether moving the faces of `b` (all, or those in `only`) `by` mm, into the material when
/// `into` (a shell's walls) else out of it (an offset growing; a negative `by` reverses either),
/// collapses a round face: a convex cylinder, sphere or torus tube of radius at most the inward
/// move, or a concave one (a hole) of radius at most the outward move.
pub fn radius_collapses(b: &Brep, only: Option<&[FaceId]>, by: f64, into: bool) -> bool {
    let inward = if into { by } else { -by };
    use crate::geom::Surface;
    b.face_ids().filter(|f| only.is_none_or(|o| o.contains(f))).any(|f| {
        let face = b.face(f);
        let r = match &face.surface {
            Surface::Cylinder(c) => c.radius,
            Surface::Sphere(s) => s.radius,
            Surface::Torus(t) => t.minor,
            _ => return false,
        };
        let convex = !face.reversed;
        if convex { inward >= r } else { -inward >= r }
    })
}

/// Whether every rim of the sheet `faces` meets its neighbour square to the sheet (the sheet's
/// normal lies in the neighbour face at the rim's midpoint), as springrep's thicken assumes when
/// it sweeps the rims along the neighbours.
pub fn rims_square(b: &Brep, faces: &[FaceId]) -> bool {
    for &f in faces {
        for &l in &b.face(f).loops {
            for &c in &b.lp(l).coedges {
                let e = b.coedge(c).edge;
                let edge = b.edge(e);
                let Some(curve) = &edge.curve else { continue };
                let p = curve.eval(0.5 * (edge.range.0 + edge.range.1));
                let Some(n) = face_normal_at(b, f, p) else { return false };
                for &u in b.edge_uses(e) {
                    let g = b.coedge_face(u);
                    if g == f || faces.contains(&g) {
                        continue;
                    }
                    let Some(m) = face_normal_at(b, g, p) else { return false };
                    if n.dot(m).abs() > crate::tol::TANGENT {
                        return false;
                    }
                }
            }
        }
    }
    true
}

/// Whether an open face of a shell meets a concave edge: there the cavity's open face stays at the
/// solid's own plane, as moved out it would cut the wall that rises past the edge back from the opening.
pub fn open_at_concave_edge(b: &Brep, open: &[FaceId]) -> bool {
    open.iter().any(|&f| {
        b.face(f).loops.iter().any(|&l| b.lp(l).coedges.iter().any(|&c| convexity(b, b.coedge(c).edge) == Some(false)))
    })
}
