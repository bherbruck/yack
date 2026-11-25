// yack: picks within a tolerance, and the nearest face (moved from springrep's facade/api.rs, M3 W10).
//! What a pick within a tolerance finds, the face nearest a point, and blend faces that stray from where
//! they were asked for: the facade's picks, over [`crate::nearest_of`].

use crate::Result;
use crate::distance::Selection;
use crate::math::{Aabb, Point3};
use crate::topo::{Brep, EdgeId, FaceId};

/// Whether `p` lies within `d` of the box.
fn near(b: &Aabb, p: Point3, d: f64) -> bool {
    p.x >= b.min.x - d
        && p.y >= b.min.y - d
        && p.z >= b.min.z - d
        && p.x <= b.max.x + d
        && p.y <= b.max.y + d
        && p.z <= b.max.z + d
}

/// What a pick within a tolerance found.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Pick<T> {
    /// Nothing within the tolerance.
    None,
    /// One entity, nearer than any other by at least the tolerance.
    One(T),
    /// This many entities within the tolerance whose distances differ by less than it: no telling
    /// which is meant.
    Ambiguous(usize),
}

fn pick<T: Copy>(mut found: Vec<(f64, T)>, d: f64) -> Pick<T> {
    found.sort_by(|a, b| a.0.total_cmp(&b.0));
    match found.as_slice() {
        [] => Pick::None,
        [(_, one)] => Pick::One(*one),
        [(d0, one), (d1, _), ..] => {
            if d1 - d0 >= d {
                Pick::One(*one)
            } else {
                Pick::Ambiguous(found.iter().filter(|(x, _)| x - d0 < d).count())
            }
        }
    }
}

/// The face of `b` within `d` of `p`, by the pick rule.
pub fn face_within(b: &Brep, p: Point3, d: f64) -> Result<Pick<FaceId>> {
    let mut found = Vec::new();
    for f in b.face_ids() {
        if !near(&crate::topo::bbox::face_bbox(b, f), p, d) {
            continue;
        }
        let dist = crate::nearest_of(b, &Selection::face(f))?.to(p)?.distance;
        if dist <= d {
            found.push((dist, f));
        }
    }
    Ok(pick(found, d))
}

/// Whether every use of `e` is by one face: a seam, where a closed surface meets itself.
fn is_seam(b: &Brep, e: EdgeId) -> bool {
    let mut faces = b.edge_uses(e).iter().map(|&c| b.coedge_face(c));
    faces.next().is_some_and(|f| faces.all(|g| g == f))
}

/// The edge of `b` (poles aside) within `d` of `p`, by the pick rule. Seams count only when no
/// other edge is in reach: a point where a rim circle meets a cylinder's seam picks the rim.
pub fn edge_within(b: &Brep, p: Point3, d: f64) -> Result<Pick<EdgeId>> {
    let (mut found, mut seams) = (Vec::new(), Vec::new());
    for e in b.edge_ids() {
        if b.edge(e).is_degenerate() {
            continue;
        }
        let dist = crate::nearest_of(b, &Selection::edge(e))?.to(p)?.distance;
        if dist <= d {
            if is_seam(b, e) { seams.push((dist, e)) } else { found.push((dist, e)) }
        }
    }
    Ok(pick(if found.is_empty() { seams } else { found }, d))
}

/// The nearest point of face `f` to `p` (the face alone, wherever the rest of the solid lies).
pub fn face_closest(b: &Brep, f: FaceId, p: Point3) -> Result<Point3> {
    Ok(crate::nearest_of(b, &Selection::face(f))?.to(p)?.on_b)
}

/// The face nearest `p` and its distance; ties (a point nearest a shared edge) go to the face whose
/// own region holds the foot of `p` on its surface, as near as it does: the one whose nearest point
/// is least farther than its surface's. Just inside a hole's rim, in the plane of the top, that is
/// the hole's wall, not the top whose surface runs through the point where the top is not.
pub fn nearest_face(b: &Brep, p: Point3) -> Result<Option<(FaceId, f64)>> {
    let mut best: Option<(f64, f64, FaceId)> = None;
    for f in b.face_ids() {
        let d = crate::nearest_of(b, &Selection::face(f))?.to(p)?.distance;
        let s = &b.face(f).surface;
        let uv = s.project(p);
        let off = s.eval(uv.x, uv.y).dist(p);
        let better = match best {
            None => true,
            Some((bd, boff, _)) => d < bd - crate::tol::LINEAR || (d <= bd + crate::tol::LINEAR && d - off < bd - boff),
        };
        if better {
            best = Some((d, off, f));
        }
    }
    Ok(best.map(|(d, _, f)| (f, d)))
}

/// Whether the blend `feature` made in `out` strays from the given edges: some connected group of
/// its blend faces (faces of `feature` whose part starts `part`, joined where they share an edge)
/// comes no nearer than `reach` to any of the given edges' midpoints `near`. A fillet of one rim
/// that also rounded another rim makes such a group (brep-bench edit/half-torus-outer-rim-round-deleted).
pub fn blend_strays(out: &Brep, feature: &str, part: &str, near: &[Point3], reach: f64) -> bool {
    if near.is_empty() {
        return false;
    }
    let blend: Vec<FaceId> = out
        .face_ids()
        .filter(|&f| out.face(f).names.iter().any(|n| n.feature == feature && n.face.starts_with(part)))
        .collect();
    let edges_of = |f: FaceId| -> std::collections::BTreeSet<EdgeId> {
        out.face(f).loops.iter().flat_map(|&l| out.lp(l).coedges.iter().map(|&c| out.coedge(c).edge)).collect()
    };
    let sets: Vec<std::collections::BTreeSet<EdgeId>> = blend.iter().map(|&f| edges_of(f)).collect();
    let mut group: Vec<usize> = (0..blend.len()).collect();
    fn root(g: &mut [usize], i: usize) -> usize {
        let mut r = i;
        while g.get(r).copied().unwrap_or(r) != r {
            r = g.get(r).copied().unwrap_or(r);
        }
        r
    }
    for i in 0..blend.len() {
        for j in (i + 1)..blend.len() {
            if let (Some(a), Some(b)) = (sets.get(i), sets.get(j))
                && !a.is_disjoint(b)
            {
                let (ri, rj) = (root(&mut group, i), root(&mut group, j));
                if let Some(slot) = group.get_mut(ri) {
                    *slot = rj;
                }
            }
        }
    }
    let mut best: std::collections::BTreeMap<usize, f64> = std::collections::BTreeMap::new();
    for (i, &f) in blend.iter().enumerate() {
        let d = near
            .iter()
            .filter_map(|&p| crate::nearest_of(out, &Selection::face(f)).ok()?.to(p).ok().map(|c| c.distance))
            .fold(f64::INFINITY, f64::min);
        let r = root(&mut group, i);
        let e = best.entry(r).or_insert(f64::INFINITY);
        *e = e.min(d);
    }
    best.values().any(|&d| d > reach)
}
