// yack: coincident vertices, the shell each vertex is in, and where a solid touches itself.
// Adapted from springrep@a03e8a1 src/check.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/check.rs to yack-check (M3 W6); split out of lib.rs into its
// own module (M3 hardening); its maps and sets are BTreeMap/BTreeSet (no order reaches an output).
//! Coincident vertices, the shell each vertex is in, and where a solid touches itself.

use std::collections::{BTreeMap, BTreeSet};

use crate::{
    tol,
    topo::{Brep, EdgeId, FaceId, ShellId, VertexId, fans_at_vertices},
};

/// The shell each vertex belongs to, by the faces whose edges end there.
pub(crate) fn vertex_shells(b: &Brep) -> BTreeMap<VertexId, ShellId> {
    // D26: BTree* in hardening, proven by digests
    let mut out = BTreeMap::new();
    for f in b.face_ids() {
        let Some(sh) = b.face(f).shell else { continue };
        for &lp in &b.face(f).loops {
            for &c in &b.lp(lp).coedges {
                let e = b.edge(b.coedge(c).edge);
                out.insert(e.start, sh);
                out.insert(e.end, sh);
            }
        }
    }
    out
}

/// Pairs of distinct vertices closer than the linear tolerance.
pub(crate) fn coincident_vertices(b: &Brep) -> Vec<(VertexId, VertexId)> {
    // D26: BTree* in hardening, proven by digests
    let cell = |x: f64| (x / (4.0 * tol::LINEAR)).floor() as i64;
    let mut grid: BTreeMap<[i64; 3], Vec<VertexId>> = BTreeMap::new();
    let mut out = Vec::new();
    for v in b.vertex_ids() {
        let p = b.vertex(v).point;
        if !(p.x.is_finite() && p.y.is_finite() && p.z.is_finite()) {
            continue;
        }
        let c = [cell(p.x), cell(p.y), cell(p.z)];
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    for &w in grid.get(&[c[0] + dx, c[1] + dy, c[2] + dz]).into_iter().flatten() {
                        if b.vertex(w).point.dist(p) <= tol::LINEAR {
                            out.push((w, v));
                        }
                    }
                }
            }
        }
        grid.entry(c).or_default().push(v);
    }
    // A tolerant vertex coincides with any within the sum of the two tolerances, which the grid's
    // cells are too small to find: a sweep along x finds those pairs.
    let (tolerant, _) = b.tolerant();
    if tolerant.is_empty() {
        return out;
    }
    let widest = tolerant.iter().map(|&(_, t)| t).fold(tol::LINEAR, f64::max);
    let mut by_x: Vec<(f64, VertexId)> =
        b.vertex_ids().map(|v| (b.vertex(v).point.x, v)).filter(|(x, _)| x.is_finite()).collect();
    by_x.sort_by(|a, b| a.0.total_cmp(&b.0));
    for &(v, tv) in &tolerant {
        let p = b.vertex(v).point;
        let reach = tol::coincidence(tv, widest);
        let from = by_x.partition_point(|&(x, _)| x < p.x - reach);
        for &(x, w) in &by_x[from..] {
            if x > p.x + reach {
                break;
            }
            if w == v || (b.vertex_tolerance(w) > tol::LINEAR && w < v) {
                continue;
            }
            let d = b.vertex(w).point.dist(p);
            if d > tol::LINEAR && d <= tol::coincidence(tv, b.vertex_tolerance(w)) {
                out.push((v.min(w), v.max(w)));
            }
        }
    }
    out
}

/// How many fans of faces meet at each vertex: following each corner there across the edge it
/// leaves by, to its partner, reaches the corners of one fan. More than one is a point where
/// pieces of the solid touch.
pub(crate) fn vertex_fans(b: &Brep) -> BTreeMap<VertexId, usize> {
    // D26: BTree* in hardening, proven by digests
    fans_at_vertices(b).into_iter().map(|(v, fs)| (v, fs.len())).collect()
}

/// Where the solid touches itself: the edges more than two faces use, and the vertices those end
/// at or that more than one fan of faces meets at.
pub fn contacts(b: &Brep) -> (BTreeSet<EdgeId>, BTreeSet<VertexId>) {
    // D26: BTree* in hardening, proven by digests
    let edges: BTreeSet<EdgeId> = b.radial_edges().collect();
    let mut vertices: BTreeSet<VertexId> =
        vertex_fans(b).into_iter().filter(|&(_, n)| n > 1).map(|(v, _)| v).collect();
    for &e in &edges {
        vertices.extend([b.edge(e).start, b.edge(e).end]);
    }
    (edges, vertices)
}

/// The faces of `faces` that run up to where the solid touches itself.
pub fn faces_at_contacts(b: &Brep, faces: impl IntoIterator<Item = FaceId>) -> Vec<FaceId> {
    let (edges, vertices) = contacts(b);
    if edges.is_empty() && vertices.is_empty() {
        return Vec::new();
    }
    faces
        .into_iter()
        .filter(|&f| {
            b.face(f).loops.iter().any(|&lp| {
                b.lp(lp).coedges.iter().any(|&c| {
                    edges.contains(&b.coedge(c).edge) || vertices.contains(&b.coedge_start(c))
                })
            })
        })
        .collect()
}
