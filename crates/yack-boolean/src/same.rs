// yack: Whether two solids are one solid: every vertex, edge and face of one coincides with one of the
// Adapted from springrep@a03e8a1 src/boolean/same.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/boolean/same.rs to yack-boolean (M3 W8); paths and visibility only.
//! Whether two solids are one solid: every vertex, edge and face of one coincides with one of the
//! other's, within the two entities' tolerances, decided in order of dimension (D. J. Jackson,
//! "Boundary representation modelling with local tolerances", Solid Modeling '95, section 4). A
//! boolean takes such a pair from its topology instead of
//! intersecting faces that lie on each other everywhere, which is where tracing is least stable.

#[allow(clippy::disallowed_types)] use std::collections::HashMap; // D26: BTree* in hardening, proven by digests

use crate::{
    intersect::{csi, ssi::same_domain},
    topo::{domain::Domain, face_domain},
};
use crate::{
    tol,
    topo::{Brep, EdgeId, FaceId, VertexId},
};

/// Whether `x` and `y`, each an arena holding one solid, are the same solid within tolerance.
pub(super) fn same_solid(x: &Brep, y: &Brep) -> bool {
    if x.face_count() != y.face_count()
        || x.edge_ids().count() != y.edge_ids().count()
        || x.vertex_ids().count() != y.vertex_ids().count()
    {
        return false;
    }
    let Some(vmap) = match_vertices(x, y) else {
        return false;
    };
    let Some(emap) = match_edges(x, y, &vmap) else {
        return false;
    };
    match_faces(x, y, &emap)
}

fn widest_vertex(b: &Brep) -> f64 {
    b.tolerant()
        .0
        .iter()
        .map(|t| t.1)
        .fold(tol::LINEAR, f64::max)
}

/// Each vertex of `x` paired with the nearest unpaired vertex of `y` within the sum of their
/// tolerances. Two vertices of one valid solid lie farther apart than that, so the nearest is the
/// only candidate.
fn match_vertices(x: &Brep, y: &Brep) -> Option<Vec<VertexId>> {
    let reach = widest_vertex(x) + widest_vertex(y);
    let mut ys: Vec<(f64, VertexId)> = y.vertex_ids().map(|v| (y.vertex(v).point.x, v)).collect();
    ys.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut taken = vec![false; ys.len()];
    let mut map = Vec::with_capacity(ys.len());
    for v in x.vertex_ids() {
        let p = x.vertex(v).point;
        let tv = x.vertex_tolerance(v);
        let lo = ys.partition_point(|w| w.0 < p.x - reach);
        let mut best: Option<(f64, usize)> = None;
        for (k, &(wx, w)) in ys.iter().enumerate().skip(lo) {
            if wx > p.x + reach {
                break;
            }
            let d = y.vertex(w).point.dist(p);
            if !taken[k]
                && crate::math::within(d, tol::coincidence(tv, y.vertex_tolerance(w)), p)
                && best.is_none_or(|b| d < b.0)
            {
                best = Some((d, k));
            }
        }
        let (_, k) = best?;
        taken[k] = true;
        map.push(ys[k].1);
    }
    Some(map)
}

/// Each edge of `x` paired with an edge of `y` between the paired vertices whose curve runs within
/// the sum of the two edges' tolerances of its own.
#[allow(clippy::disallowed_types)] fn match_edges(x: &Brep, y: &Brep, vmap: &[VertexId]) -> Option<Vec<EdgeId>> { // D26: BTree* in hardening, proven by digests
    let ends = |a: VertexId, b: VertexId| (a.min(b), a.max(b));
    let mut by_ends: HashMap<(VertexId, VertexId), Vec<EdgeId>> = HashMap::new();
    for e in y.edge_ids() {
        let edge = y.edge(e);
        by_ends
            .entry(ends(edge.start, edge.end))
            .or_default()
            .push(e);
    }
    let mut taken = vec![false; y.edge_ids().count()];
    let mut map = Vec::with_capacity(taken.len());
    for e in x.edge_ids() {
        let edge = x.edge(e);
        let key = ends(vmap[edge.start.index()], vmap[edge.end.index()]);
        let found = by_ends.get(&key)?.iter().copied().find(|&f| {
            if taken[f.index()] {
                return false;
            }
            let other = y.edge(f);
            let reach = tol::coincidence(x.edge_tolerance(e), y.edge_tolerance(f));
            match (&edge.curve, &other.curve) {
                (None, None) => true,
                (Some(c), Some(d)) => {
                    let on = |c: &crate::geom::Curve, r: (f64, f64), d, s: (f64, f64)| {
                        (1..4).all(|k| {
                            let p = c.eval(r.0 + (r.1 - r.0) * k as f64 / 4.0);
                            crate::math::within(csi::curve_distance(d, s, p).0, reach, p)
                        })
                    };
                    on(c, edge.range, d, other.range) && on(d, other.range, c, edge.range)
                }
                _ => false,
            }
        })?;
        taken[found.index()] = true;
        map.push(found);
    }
    Some(map)
}

/// Each face of `x` paired with a face of `y` bounded by the paired edges, lying on it and facing
/// the same way.
#[allow(clippy::disallowed_types)] fn match_faces(x: &Brep, y: &Brep, emap: &[EdgeId]) -> bool { // D26: BTree* in hardening, proven by digests
    let edges_of = |b: &Brep, f: FaceId, map: &dyn Fn(EdgeId) -> EdgeId| {
        let mut es: Vec<EdgeId> = b
            .face(f)
            .loops
            .iter()
            .flat_map(|&l| b.lp(l).coedges.iter().map(|&c| map(b.coedge(c).edge)))
            .collect();
        es.sort();
        es
    };
    let mut by_edges: HashMap<Vec<EdgeId>, Vec<FaceId>> = HashMap::new();
    for f in y.face_ids() {
        by_edges.entry(edges_of(y, f, &|e| e)).or_default().push(f);
    }
    let mut taken = vec![false; y.face_count()];
    for f in x.face_ids() {
        let Some(candidates) = by_edges.get(&edges_of(x, f, &|e| emap[e.index()])) else {
            return false;
        };
        let dx = face_domain(x, f);
        let found = candidates.iter().copied().find(|&g| {
            !taken[g.index()] && {
                let dy = face_domain(y, g);
                on_each_other(&dx, &dy) && on_each_other(&dy, &dx)
            }
        });
        match found {
            Some(g) => taken[g.index()] = true,
            None => return false,
        }
    }
    true
}

/// Whether points well inside face `a` lie inside face `b`, on its surface within the tolerance,
/// with the same outward normal. Their boundaries already coincide, so this only tells faces on one
/// surface from faces on two that share their edges. A spline's parameter window can reach past the
/// face to a pole where projection is ill-conditioned, so only the inside is read.
fn on_each_other(a: &Domain, b: &Domain) -> bool {
    let mut pts = a.interior_points();
    if pts.is_empty() {
        pts.extend(a.interior_point());
    }
    let n = if same_domain(&a.surface, &b.surface) {
        1
    } else {
        8
    };
    !pts.is_empty()
        && pts.iter().take(n).all(|&uv| {
            let p = a.surface.eval(uv.x, uv.y);
            let w = b.surface.project(p);
            crate::math::within(b.surface.eval(w.x, w.y).dist(p), tol::LINEAR, p)
                && b.exact_side(p)
                    .is_some_and(|w| b.outward(w).dot(a.outward(uv)) > 0.0)
        })
}
