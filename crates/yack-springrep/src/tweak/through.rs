// yack: A move of faces followed from where they are to where they go, its topology changing where a
// Adapted from springrep@a03e8a1 src/edit/tweak/through.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from edit/tweak/through.rs to tweak/through.rs (M2.4, T6); paths only.
//! A move of faces followed from where they are to where they go, its topology changing where a
//! face closes on the way: walls drafted until they meet close over the top between them, a step
//! raised level with the top closes the riser, and a round offset past its radius closes into the
//! corner it rounded.
//!
//! The faces move along paths from their old surfaces to their new ones. The furthest the solid can
//! go with its topology kept is found by bisection; there each edge that has shrunk to nothing is
//! collapsed to a vertex, and each face left a strip between two edges that now run together is
//! taken out, the two edges one. The move then goes on from there. An event that needs more, as an
//! edge that would turn through a vertex of four faces, ends it with the error the move gave.

use std::collections::HashMap;

use super::{move_faces_pinned, near_uv};
use crate::{
    Result,
    geom::{Cone, Surface},
    math::{Frame, Vec2},
    tol,
    topo::{Brep, CoedgeSpec, EdgeId, FaceId, PCurve, VertexId, radial},
};

/// A face's surface a share `s` of the way along its move, `s` from 0 to 1; an error where it has
/// no surface there, as a cylinder offset past its radius.
pub(crate) type Path<'a> = Box<dyn Fn(f64) -> Result<Surface> + 'a>;

/// The plane a share `s` of the way from `from` to `to` through the pencil of planes they span, so
/// a plane tilted about a hinge keeps the hinge and one moved along its normal stays parallel.
pub(crate) fn pencil(from: &Frame, to: &Frame, s: f64) -> Result<Frame> {
    let (c0, c1) = (from.z.dot(from.origin), to.z.dot(to.origin));
    let n = from.z * (1.0 - s) + to.z * s;
    let k = n.norm();
    let (n, c) = (n / k, (c0 * (1.0 - s) + c1 * s) / k);
    let origin = from.origin + n * (c - n.dot(from.origin));
    Frame::new(origin, n, from.x)
}

/// The cylinder `from` tilted into the cone `to` about the circle where they meet, a share `s` of
/// the way: the cone through that circle at `s` of the half angle.
pub(crate) fn opening(to: &Cone, s: f64) -> Surface {
    Surface::Cone(Cone {
        half_angle: to.half_angle * s,
        ..to.clone()
    })
}

/// How short an edge is, in mm, before the move takes it to have closed.
const CLOSED: f64 = 10.0 * tol::LINEAR;

/// `b` with the faces of `paths` moved to the ends of their paths, its topology changed where a
/// face closes on the way, and whether one did: the faces round it are then for the caller to merge
/// (`boolean::unify`, which this layer may not call).
pub(crate) fn through(b: &Brep, paths: &HashMap<FaceId, Path>) -> Result<(Brep, bool)> {
    let mut cur = b.clone();
    // The face of `b` each face of `cur` comes from.
    let mut from: HashMap<FaceId, FaceId> = b.face_ids().map(|f| (f, f)).collect();
    let mut s0 = 0.0;
    let mut changed = false;
    loop {
        let at = |s: f64| -> Result<Brep> {
            let mut moved = HashMap::new();
            for f in cur.face_ids() {
                if let Some(p) = paths.get(&from[&f]) {
                    moved.insert(f, p(s)?);
                } else if changed {
                    // After a collapse every vertex settles again where its faces meet, those the
                    // closed faces carried as well.
                    moved.insert(f, cur.face(f).surface.clone());
                }
            }
            move_faces_pinned(&cur, &moved, &HashMap::new())
        };
        let last = match at(1.0) {
            Ok(out) => return Ok((out, changed)),
            Err(e) => e,
        };
        let (mut lo, mut hi) = (s0, 1.0);
        while hi - lo > 1e-15 * (1.0 + hi) {
            let mid = 0.5 * (lo + hi);
            if mid <= lo || mid >= hi {
                break;
            }
            if at(mid).is_ok() {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        let Ok(near) = at(lo) else {
            return Err(last);
        };
        let Some((next, kept)) = collapse(&near, CLOSED) else {
            return Err(last);
        };
        // Faces of `near` are `cur`'s in order; each kept one maps on to its face of `next`.
        let next_from = cur
            .face_ids()
            .zip(&kept)
            .filter_map(|(f, nf)| nf.map(|nf| (nf, from[&f])))
            .collect();
        cur = next;
        from = next_from;
        s0 = lo;
        changed = true;
    }
}

fn root(p: &mut [usize], mut i: usize) -> usize {
    while p[i] != i {
        p[i] = p[p[i]];
        i = p[i];
    }
    i
}

/// `b` with what has closed taken out: vertices within `eps` of each other made one, edges then
/// running together made one, and edges shorter than `eps` gone; faces left a strip along one edge,
/// or lying on each other face to face, taken out; and a face left running along an edge and back
/// that nothing else uses rid of it. With, for each face of `b`, its face in the result. `None`
/// where nothing has closed.
fn collapse(b: &Brep, eps: f64) -> Option<(Brep, Vec<Option<FaceId>>)> {
    let eids: Vec<EdgeId> = b.edge_ids().collect();
    let vids: Vec<VertexId> = b.vertex_ids().collect();
    let nv = vids.len();
    let mut parent: Vec<usize> = (0..nv).collect();
    let mut changed = false;
    let mut order: Vec<VertexId> = b.vertex_ids().collect();
    order.sort_by(|x, y| b.vertex(*x).point.x.total_cmp(&b.vertex(*y).point.x));
    for (k, &v) in order.iter().enumerate() {
        let p = b.vertex(v).point;
        for &w in &order[k + 1..] {
            let q = b.vertex(w).point;
            if q.x - p.x > eps {
                break;
            }
            if p.dist(q) <= eps {
                let (a, c) = (root(&mut parent, v.index()), root(&mut parent, w.index()));
                if a != c {
                    parent[a.max(c)] = a.min(c);
                    changed = true;
                }
            }
        }
    }
    let at = |p: &mut Vec<usize>, v: VertexId| root(p, v.index());
    // Each edge's replacement and whether it runs the other way; `None` for one closed to a point.
    let ne = b.edge_ids().count();
    let mut into: Vec<Option<(usize, bool)>> = (0..ne).map(|i| Some((i, false))).collect();
    let mut mids = Vec::with_capacity(ne);
    for e in b.edge_ids() {
        let x = b.edge(e);
        mids.push(
            x.curve
                .as_ref()
                .map(|c| c.d1(0.5 * (x.range.0 + x.range.1))),
        );
        if let Some(c) = &x.curve
            && radial::length(c, x.range) <= eps
        {
            into[e.index()] = None;
            changed = true;
        }
    }
    let mut by_ends: HashMap<(usize, usize), Vec<usize>> = HashMap::new();
    for e in b.edge_ids() {
        let x = b.edge(e);
        if into[e.index()].is_none() || x.curve.is_none() {
            continue;
        }
        let (s, t) = (at(&mut parent, x.start), at(&mut parent, x.end));
        by_ends
            .entry((s.min(t), s.max(t)))
            .or_default()
            .push(e.index());
    }
    for group in by_ends.values().filter(|g| g.len() > 1) {
        for (k, &i) in group.iter().enumerate() {
            if into[i] != Some((i, false)) {
                continue;
            }
            let (m, d) = mids[i].expect("curved");
            for &j in &group[k + 1..] {
                if into[j] != Some((j, false)) {
                    continue;
                }
                let y = b.edge(eids[j]);
                let c = y.curve.as_ref().expect("curved");
                let t = c.project(m, y.range.0, y.range.1);
                let (q, dq) = c.d1(t);
                if q.dist(m) <= eps {
                    into[j] = Some((i, d.dot(dq) < 0.0));
                    changed = true;
                }
            }
        }
    }
    if !changed {
        return None;
    }
    // Each face's loops as (edge, reversed) after the merges, closed edges left out.
    let mut loops: Vec<Vec<Vec<(usize, bool, crate::topo::CoedgeId)>>> = b
        .face_ids()
        .map(|f| {
            b.face(f)
                .loops
                .iter()
                .map(|&lp| {
                    b.lp(lp)
                        .coedges
                        .iter()
                        .filter_map(|&c| {
                            let co = b.coedge(c);
                            into[co.edge.index()].map(|(e, flip)| (e, co.reversed != flip, c))
                        })
                        .collect::<Vec<_>>()
                })
                .filter(|l: &Vec<_>| !l.is_empty())
                .collect()
        })
        .collect();
    let nf = loops.len();
    let mut gone = vec![false; nf];
    let edge_set = |l: &Vec<Vec<(usize, bool, crate::topo::CoedgeId)>>| {
        let mut s: Vec<usize> = l.iter().flatten().map(|x| x.0).collect();
        s.sort_unstable();
        s
    };
    for f in 0..nf {
        let es = edge_set(&loops[f]);
        // No boundary, or a strip along one edge.
        if es.is_empty() || (es.len() == 2 && es[0] == es[1] && loops[f].len() == 1) {
            gone[f] = true;
        }
    }
    // Faces face to face: the same edges, one lying on the other.
    let faces: Vec<FaceId> = b.face_ids().collect();
    for f in 0..nf {
        for g in f + 1..nf {
            if gone[f] || gone[g] || edge_set(&loops[f]) != edge_set(&loops[g]) {
                continue;
            }
            let dom = crate::locate::face_domain(b, faces[f]);
            let Some(uv) = dom.interior_point() else {
                continue;
            };
            let p = dom.surface.eval(uv.x, uv.y);
            let on = &b.face(faces[g]).surface;
            if crate::intersect::csi::signed_distance(on, p).abs() <= eps {
                gone[f] = true;
                gone[g] = true;
            }
        }
    }
    // A face running along an edge and back that no other face uses: a slit, which is no edge.
    let mut uses: HashMap<usize, usize> = HashMap::new();
    for f in (0..nf).filter(|&f| !gone[f]) {
        for &(e, _, _) in loops[f].iter().flatten() {
            *uses.entry(e).or_default() += 1;
        }
    }
    let alive: Vec<usize> = (0..nf).filter(|&f| !gone[f]).collect();
    for f in alive {
        for l in &mut loops[f] {
            loop {
                let n = l.len();
                let found = (0..n).find(|&k| {
                    let (a, c) = (l[k], l[(k + 1) % n]);
                    n >= 2 && a.0 == c.0 && a.1 != c.1 && uses[&a.0] == 2
                });
                let Some(k) = found else { break };
                let j = (k + 1) % n;
                let (hi, lo) = (k.max(j), k.min(j));
                l.remove(hi);
                l.remove(lo);
            }
        }
        loops[f].retain(|l| !l.is_empty());
        if loops[f].is_empty() {
            gone[f] = true;
        }
    }
    let used: std::collections::BTreeSet<usize> = (0..nf)
        .filter(|&f| !gone[f])
        .flat_map(|f| loops[f].iter().flatten().map(|x| x.0).collect::<Vec<_>>())
        .collect();
    let mut out = Brep::new();
    let mut verts: HashMap<usize, VertexId> = HashMap::new();
    let mut vertex = |out: &mut Brep, p: &mut Vec<usize>, v: VertexId| -> VertexId {
        let r = root(p, v.index());
        *verts
            .entry(r)
            .or_insert_with(|| out.add_vertex(b.vertex(vids[r]).point))
    };
    let mut edges: HashMap<usize, EdgeId> = HashMap::new();
    for &i in &used {
        let x = b.edge(eids[i]);
        let s = vertex(&mut out, &mut parent, x.start);
        let t = vertex(&mut out, &mut parent, x.end);
        edges.insert(
            i,
            match &x.curve {
                Some(c) => out.add_edge(c.clone(), x.range, s, t),
                None => out.add_degenerate_edge(s, x.range),
            },
        );
    }
    let mut kept = vec![None; nf];
    for f in 0..nf {
        if gone[f] {
            continue;
        }
        let face = b.face(faces[f]);
        let id = out.add_face(face.surface.clone(), face.reversed, face.names.clone());
        for l in &loops[f] {
            let mut last: Option<Vec2> = None;
            let mut specs = Vec::new();
            for &(e, reversed, c) in l {
                let edge = edges[&e];
                let x = out.edge(edge);
                let pcurve = match &x.curve {
                    None => b.coedge(c).pcurve.clone(),
                    Some(cv) => {
                        let from = if reversed { x.end } else { x.start };
                        let seed = near_uv(&face.surface, out.vertex(from).point, last);
                        let (t0, t1) = if reversed {
                            (x.range.1, x.range.0)
                        } else {
                            x.range
                        };
                        let mut uv = seed;
                        for k in 1..=32 {
                            let p = cv.eval(t0 + (t1 - t0) * k as f64 / 32.0);
                            uv = near_uv(&face.surface, p, Some(uv));
                        }
                        last = Some(uv);
                        PCurve::Projected { seed }
                    }
                };
                specs.push(CoedgeSpec {
                    edge,
                    reversed,
                    pcurve,
                });
            }
            out.add_loop(id, specs);
        }
        kept[f] = Some(id);
    }
    for sh in b.shell_ids() {
        let ids: Vec<FaceId> = b
            .shell(sh)
            .faces
            .iter()
            .filter_map(|f| kept[f.index()])
            .collect();
        let _ = out.add_shell(ids);
    }
    for so in b.solid_ids() {
        out.add_solid_of(b.solid(so).shells.clone(), b.solid(so).outer);
    }
    out.pair_radial();
    Some((out, kept))
}
