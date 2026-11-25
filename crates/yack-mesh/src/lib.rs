// yack: Tessellation. Each edge is discretised once and both its faces use those points, so the mesh of
// Adapted from springrep@a03e8a1 src/mesh/mod.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/mesh/mod.rs to yack-mesh (M3 W6); paths and visibility only;
// hardening (w-h1): ordered maps, checked access, par_map over a slice, a bad vertex an error.
//! Tessellation. Each edge is discretised once and both its faces use those points, so the mesh of
//! a valid solid is watertight by construction. Each face is a constrained Delaunay triangulation
//! of those points in its parameter plane, refined until every triangle is within the deflection
//! limits of the surface (`face`).


// springrep's foundation modules (D26), under the names mesh's code uses.
use yack_base::kernel::Error;
use yack_base::kernel::Result;
use yack_locate::distance;
use yack_base::kernel as error;
use yack_geom as geom;
use yack_intersect as intersect;
use yack_math::kernel as math;
use yack_locate::rays;
use yack_tol as tol;
use yack_topo as topo;
mod cdt;
pub mod display;
mod face;
mod predicates;
mod query;
#[cfg(test)]
mod tests;

pub use crate::geom::{MeshOptions, edge_params};
pub use query::{closest, nearest_boundary, nearest_of, nearest_solid, rays};

use std::collections::{BTreeMap, BTreeSet};
use std::ops::Range;
use std::sync::atomic::{AtomicUsize, Ordering};

use cdt::BOX;
use predicates::orient;

use crate::{
    geom::Curve,
    math::{Point3, Vec2, Vec3},
    topo::{Brep, CoedgeId, EdgeId, FaceId, SolidId},
};



#[derive(Clone, Debug, Default, PartialEq)]
pub struct Mesh {
    pub positions: Vec<Point3>,
    pub triangles: Vec<[u32; 3]>,
    /// Each face's triangles, as a range of `triangles`.
    pub faces: Vec<(FaceId, Range<usize>)>,
}

/// `opt` with its deflection between a ten-thousandth and a thousandth of `b`'s size: for a mesh
/// that seeds exact searches over `b` (rays, distances), whose triangles must stand near the faces
/// however small the part, and need not be finer than the search's own margin however large.
pub(crate) fn seeding(opt: &MeshOptions, b: &Brep) -> MeshOptions {
    let size = crate::topo::bbox::loose_bbox(b).diagonal();
    MeshOptions {
        linear: if size > 0.0 {
            opt.linear.clamp(1e-4 * size, 1e-3 * size)
        } else {
            opt.linear
        },
        angular: opt.angular,
    }
}





/// Tessellates every face of `b` within `opt`.
pub fn tessellate(b: &Brep, opt: &MeshOptions) -> Result<Mesh> {
    let positive = |x: f64| x > 0.0 && x.is_finite();
    if !positive(opt.linear) || !positive(opt.angular) {
        return Err(Error::InvalidInput(format!(
            "mesh deflections must be positive and finite: linear {}, angular {}",
            opt.linear, opt.angular
        )));
    }
    let mut mesh = Mesh::default();
    let vgid: Vec<u32> = b
        .vertex_ids()
        .map(|v| {
            mesh.positions.push(b.vertex(v).point);
            (mesh.positions.len() - 1) as u32
        })
        .collect();
    let mut uses: Vec<Vec<CoedgeId>> = vec![Vec::new(); b.edge_ids().count()];
    for c in b.coedge_ids() {
        if let Some(on_edge) = uses.get_mut(b.coedge(c).edge.index()) {
            on_edge.push(c);
        }
    }
    let edges: Vec<EdgeId> = b.edge_ids().collect();
    let jobs: Vec<(EdgeId, &[CoedgeId])> =
        edges.iter().copied().zip(uses.iter().map(Vec::as_slice)).collect();
    let params = par_map(&jobs, |&(e, coedges)| {
        let edge = b.edge(e);
        let (t0, t1) = edge.range;
        match &edge.curve {
            Some(c) => fit_faces(b, c, edge_params(c, t0, t1, opt), coedges, opt),
            None => {
                let n = ((t1 - t0) / opt.angular).ceil().max(1.0) as usize;
                (0..=n)
                    .map(|k| t0 + (t1 - t0) * k as f64 / n as f64)
                    .collect()
            }
        }
    });
    let mut params = params;
    // Two edges between one pair of vertices, each drawn as one chord, would draw one segment
    // that all four of their faces use.
    let mut by_ends: BTreeMap<(usize, usize), Vec<usize>> = BTreeMap::new();
    for (i, (&e, ts)) in edges.iter().zip(&params).enumerate() {
        let edge = b.edge(e);
        if edge.curve.is_some() && ts.len() == 2 && edge.start != edge.end {
            let (a, c) = (edge.start.index(), edge.end.index());
            by_ends.entry((a.min(c), a.max(c))).or_default().push(i);
        }
    }
    for group in by_ends.values().filter(|g| g.len() > 1) {
        for &i in group {
            if let Some(ts) = params.get_mut(i)
                && let &[t0, t1] = ts.as_slice()
            {
                ts.insert(1, 0.5 * (t0 + t1));
            }
        }
    }
    let vertex_gid = |v: crate::topo::VertexId| {
        vgid.get(v.index()).copied().ok_or_else(|| {
            Error::InvalidInput(format!("an edge's vertex {} is not in the solid", v.index()))
        })
    };
    let mut samples: Vec<Vec<(f64, u32)>> = Vec::with_capacity(edges.len());
    for (&e, ts) in edges.iter().zip(params) {
        let edge = b.edge(e);
        let last = ts.len().saturating_sub(1);
        let s = ts
            .iter()
            .enumerate()
            .map(|(k, &t)| {
                let gid = match &edge.curve {
                    _ if k == 0 => vertex_gid(edge.start)?,
                    _ if k == last => vertex_gid(edge.end)?,
                    Some(c) => {
                        mesh.positions.push(c.eval(t));
                        (mesh.positions.len() - 1) as u32
                    }
                    None => vertex_gid(edge.start)?,
                };
                Ok((t, gid))
            })
            .collect::<Result<_>>()?;
        samples.push(s);
    }
    uncross(b, &edges, &mut samples, &mut mesh.positions);
    drop_unresolved(b, &edges, &mut samples);
    separate_twins(b, &edges, &mut samples, &mut mesh.positions);
    faces(b, opt, &samples, &mut mesh)?;
    Ok(mesh)
}

/// Splits boundary segments that cross another of the same face's segments in its parameter
/// plane. Two edges that leave nearly one point at a small angle can have chords that cross where
/// the curves do not; halving the curved one's chords near there separates them. Segments that
/// still cross after the rounds are left for the face to refuse.
fn uncross(
    b: &Brep,
    edges: &[EdgeId],
    samples: &mut [Vec<(f64, u32)>],
    positions: &mut Vec<Point3>,
) {
    let faces: Vec<FaceId> = b.face_ids().collect();
    for _ in 0..32 {
        let found = par_map(&faces, |&f| crossing_segments(b, f, samples));
        let split: BTreeSet<(usize, usize)> = found.into_iter().flatten().collect();
        if split.is_empty() {
            return;
        }
        let mut by_edge: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
        for (e, k) in split {
            by_edge.entry(e).or_default().push(k);
        }
        for (e, ks) in by_edge {
            let (Some(&edge), Some(edge_samples)) = (edges.get(e), samples.get_mut(e)) else {
                continue;
            };
            let Some(curve) = &b.edge(edge).curve else {
                continue;
            };
            // A chord no longer than the edge's tolerance (an imported edge's own, else the linear
            // one) cannot be told apart from one it crosses by halving it: its image in the face
            // is known no better, and below the linear tolerance a curve's parameter-plane points
            // are rounding, whose wobble halving would chase down to coincident samples.
            let within = b.edge_tolerance(edge).max(crate::tol::LINEAR);
            let resolved = |a: f64, z: f64| curve.eval(a).dist(curve.eval(z)) > within;
            let old = std::mem::take(edge_samples);
            let mut new = Vec::with_capacity(old.len() + ks.len());
            for (k, &here) in old.iter().enumerate() {
                new.push(here);
                if ks.contains(&k)
                    && let Some(&next) = old.get(k + 1)
                    && resolved(here.0, next.0)
                {
                    let t = 0.5 * (here.0 + next.0);
                    positions.push(curve.eval(t));
                    new.push((t, (positions.len() - 1) as u32));
                }
            }
            *edge_samples = new;
        }
    }
}

/// Gives each of two or more edges between one pair of vertices that is left a single chord (after
/// [`drop_unresolved`]) a sample at its middle: their chords would otherwise be one segment that
/// all their faces use, four triangles on one edge (bench swept/tumbling-torus-full-turn, carried).
fn separate_twins(b: &Brep, edges: &[EdgeId], samples: &mut [Vec<(f64, u32)>], positions: &mut Vec<Point3>) {
    let mut by_ends: BTreeMap<(usize, usize), Vec<usize>> = BTreeMap::new();
    for (i, (&e, s)) in edges.iter().zip(samples.iter()).enumerate() {
        let edge = b.edge(e);
        if edge.curve.is_some() && s.len() == 2 && edge.start != edge.end {
            let (a, c) = (edge.start.index(), edge.end.index());
            by_ends.entry((a.min(c), a.max(c))).or_default().push(i);
        }
    }
    for group in by_ends.values().filter(|g| g.len() > 1) {
        for &i in group {
            let (Some(&e), Some(s)) = (edges.get(i), samples.get_mut(i)) else { continue };
            let Some(curve) = &b.edge(e).curve else { continue };
            let &[(t0, _), (t1, _)] = s.as_slice() else { continue };
            let t = 0.5 * (t0 + t1);
            positions.push(curve.eval(t));
            s.insert(1, (t, (positions.len() - 1) as u32));
        }
    }
}

/// Drops the samples of an imported edge with a local tolerance that one of its faces sees at the
/// same parameters as a neighbour: the edge misses that face by up to its tolerance, and where it
/// runs past a pole, a stretch of it projects to one point. Both faces drop the sample, so they
/// still share every chord.
fn drop_unresolved(b: &Brep, edges: &[EdgeId], samples: &mut [Vec<(f64, u32)>]) {
    for (&e, edge_samples) in edges.iter().zip(samples.iter_mut()) {
        if b.edge_tolerance(e) <= crate::tol::LINEAR || edge_samples.len() < 3 {
            continue;
        }
        let ts: Vec<f64> = edge_samples.iter().map(|x| x.0).collect();
        let images: Vec<Vec<Vec2>> = b
            .coedge_ids()
            .filter(|&c| b.coedge(c).edge == e)
            .map(|c| {
                let reversed = b.coedge(c).reversed;
                let order: Vec<f64> = if reversed {
                    ts.iter().rev().copied().collect()
                } else {
                    ts.clone()
                };
                let mut uvs = b.coedge_uvs(c, &order);
                if reversed {
                    uvs.reverse();
                }
                uvs
            })
            .collect();
        let last = ts.len() - 1;
        let same = |a: usize, z: usize| {
            images.iter().any(|uvs| {
                let (Some(&ua), Some(&uz)) = (uvs.get(a), uvs.get(z)) else {
                    return false;
                };
                let near = 1e-12 * (1.0 + ua.x.abs() + ua.y.abs());
                (ua - uz).norm() <= near
            })
        };
        let mut kept = vec![0];
        for k in 1..last {
            if !same(*kept.last().unwrap_or(&0), k) && !same(k, last) {
                kept.push(k);
            }
        }
        kept.push(last);
        if kept.len() < ts.len() {
            *edge_samples = kept.into_iter().filter_map(|k| edge_samples.get(k).copied()).collect();
        }
    }
}

/// Where one coedge of a loop begins among a loop's points, and the parameters it ends at.
pub type Run = (usize, Vec2);

/// Lays a face's boundary out as one figure in the parameter plane of a periodic surface. Each
/// coedge's parameters are seeded on their own, and may lie a whole period from the coedge before
/// it: a cylinder's seam at −π under a rim that runs from π to 3π. Where every loop closes once
/// its coedges are laid end to end, they are laid so, and each other loop is moved by whole
/// periods into the largest. A loop that winds round the surface does not close, and a face with
/// one is left as it is seeded. A loop already inside the largest stays where it is. `runs` gives
/// each loop's coedges in order.
pub(crate) fn one_sheet<T>(
    periods: (Option<f64>, Option<f64>),
    loops: &mut [Vec<(Vec2, T)>],
    runs: &[Vec<Run>],
) {
    if periods.0.is_none() && periods.1.is_none() {
        return;
    }
    let whole = |d: f64, p: Option<f64>| p.map_or(0.0, |p| (d / p).round() * p);
    let turns = |d: Vec2| Vec2::new(whole(d.x, periods.0), whole(d.y, periods.1));
    let mut shifts: Vec<Vec<Vec2>> = Vec::with_capacity(loops.len());
    for (pts, run) in loops.iter().zip(runs) {
        let mut shift = Vec2::ZERO;
        let mut each = Vec::with_capacity(run.len());
        let mut end: Option<Vec2> = None;
        for &(at, last) in run {
            let Some(&(first, _)) = pts.get(at) else {
                return;
            };
            if let Some(e) = end {
                shift = turns(e - first);
            }
            each.push(shift);
            end = Some(last + shift);
        }
        // Back at its start, to well within a period: else the loop winds round the surface.
        let (Some(e), Some(&(first, _))) = (end, pts.first()) else {
            return;
        };
        if turns(e - first) != Vec2::ZERO {
            return;
        }
        shifts.push(each);
    }
    for ((pts, run), each) in loops.iter_mut().zip(runs).zip(&shifts) {
        for (k, (&(at, _), &shift)) in run.iter().zip(each).enumerate() {
            let to = run.get(k + 1).map_or(pts.len(), |next| next.0);
            for p in pts.get_mut(at..to).into_iter().flatten() {
                p.0 = p.0 + shift;
            }
        }
    }
    let area = |pts: &[(Vec2, T)]| -> f64 {
        closed_sides(pts.iter().map(|p| p.0))
            .map(|(a, b)| a.x * b.y - b.x * a.y)
            .sum::<f64>()
            .abs()
    };
    let Some((outer, outer_pts)) =
        loops.iter().enumerate().max_by(|(_, x), (_, y)| area(x).total_cmp(&area(y)))
    else {
        return;
    };
    let ring: Vec<Vec2> = outer_pts.iter().map(|p| p.0).collect();
    let inside = |q: Vec2| -> bool {
        let mut odd = false;
        for (a, b) in closed_sides(ring.iter().copied()) {
            if (a.y > q.y) != (b.y > q.y) && q.x < a.x + (q.y - a.y) / (b.y - a.y) * (b.x - a.x) {
                odd = !odd;
            }
        }
        odd
    };
    for (i, pts) in loops.iter_mut().enumerate() {
        if i == outer || pts.is_empty() {
            continue;
        }
        // A point of the loop's own, not on the outer loop's boundary: the middle of its points.
        let n = pts.len() as f64;
        let mid = pts.iter().fold(Vec2::ZERO, |s, p| s + p.0) * (1.0 / n);
        let steps = |p: Option<f64>| -> Vec<f64> {
            match p {
                Some(p) => (-3..=3).map(|k| f64::from(k) * p).collect(),
                None => vec![0.0],
            }
        };
        let mut best = None;
        for du in steps(periods.0) {
            for dv in steps(periods.1) {
                let d = Vec2::new(du, dv);
                if inside(mid + d) && best.is_none_or(|b: Vec2| d.norm() < b.norm()) {
                    best = Some(d);
                }
            }
        }
        if let Some(d) = best.filter(|d| *d != Vec2::ZERO) {
            for p in pts.iter_mut() {
                p.0 = p.0 + d;
            }
        }
    }
}

/// Each point of a closed polygon with the next, the last with the first.
fn closed_sides<P: Copy>(points: impl Iterator<Item = P> + Clone) -> impl Iterator<Item = (P, P)> {
    points.clone().zip(points.cycle().skip(1))
}

/// Moves each parameter-plane image of a mesh point onto an earlier image of the same point when
/// rounding alone parts them, so the face is meshed through one point there and not two a hair
/// apart. Images a period or a pole's span apart are far more than this apart.
pub(crate) fn snap_repeats(pts: &mut [(Vec2, u32)]) {
    let (mut lo, mut hi) = (
        Vec2::new(f64::INFINITY, f64::INFINITY),
        Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY),
    );
    for (uv, _) in pts.iter() {
        lo = Vec2::new(lo.x.min(uv.x), lo.y.min(uv.y));
        hi = Vec2::new(hi.x.max(uv.x), hi.y.max(uv.y));
    }
    let near = 1e-9 * (hi - lo).norm();
    let mut seen: BTreeMap<u32, Vec<Vec2>> = BTreeMap::new();
    for (uv, g) in pts.iter_mut() {
        let images = seen.entry(*g).or_default();
        match images.iter().find(|q| (**q - *uv).norm() <= near) {
            Some(&q) => *uv = q,
            None => images.push(*uv),
        }
    }
}

/// The (edge, segment) pairs of a face's boundary segments that cross or touch another of its
/// segments other than at a shared end, every segment of a loop too short to bound an area, and
/// every curved segment with another loop point between it and its curve, which a coarse
/// deflection leaves on the wrong side of the chord.
fn crossing_segments(b: &Brep, f: FaceId, samples: &[Vec<(f64, u32)>]) -> Vec<(usize, usize)> {
    // Each loop's points as the face mesher takes them: every coedge's samples but its last.
    let mut segs: Vec<(Vec2, Vec2, usize, usize)> = Vec::new();
    let mut neighbours: Vec<(usize, usize)> = Vec::new();
    let mut out = Vec::new();
    let mut loops: Vec<Vec<(Vec2, usize, usize)>> = Vec::new();
    let mut points: Vec<(Vec2, u32)> = Vec::new();
    let mut mids: Vec<Option<Vec2>> = Vec::new();
    let mut sheet: Vec<Vec<(Vec2, usize)>> = Vec::new();
    let mut runs: Vec<Vec<Run>> = Vec::new();
    for &lp in &b.face(f).loops {
        let mut pts: Vec<(Vec2, usize, usize)> = Vec::new();
        let mut run: Vec<Run> = Vec::new();
        for &co in &b.lp(lp).coedges {
            let c = b.coedge(co);
            let e = c.edge.index();
            let Some(edge_samples) = samples.get(e) else { continue };
            let n = edge_samples.len();
            let mut ts: Vec<f64> = edge_samples.iter().map(|x| x.0).collect();
            if c.reversed {
                ts.reverse();
            }
            let uvs = b.coedge_uvs(co, &ts);
            let (Some(&end), Some(&t_end)) = (uvs.last(), ts.last()) else { continue };
            run.push((pts.len(), end));
            let curved = matches!(&b.edge(c.edge).curve, Some(cv) if !matches!(cv, Curve::Line(_)));
            // Interleaved with the samples, so a periodic parameter continues as it does there.
            let halves: Vec<Option<Vec2>> = if curved {
                let mut both = Vec::with_capacity(2 * n);
                for pair in ts.windows(2) {
                    if let &[t0, t1] = pair {
                        both.push(t0);
                        both.push(0.5 * (t0 + t1));
                    }
                }
                both.push(t_end);
                let uv2 = b.coedge_uvs(co, &both);
                uv2.iter().skip(1).step_by(2).take(n - 1).map(|&uv| Some(uv)).collect()
            } else {
                vec![None; n - 1]
            };
            for (k, (&uv, &half)) in uvs.iter().zip(&halves).enumerate().take(n - 1) {
                let (seg, at) = if c.reversed {
                    (n - 2 - k, n - 1 - k)
                } else {
                    (k, k)
                };
                let Some(&(_, gid)) = edge_samples.get(at) else { continue };
                pts.push((uv, e, seg));
                points.push((uv, gid));
                mids.push(half);
            }
        }
        let base = points.len() - pts.len();
        sheet.push(pts.iter().enumerate().map(|(k, p)| (p.0, base + k)).collect());
        runs.push(run);
        loops.push(pts);
    }
    let surface = &b.face(f).surface;
    one_sheet((surface.u_period(), surface.v_period()), &mut sheet, &runs);
    for &(uv, k) in sheet.iter().flatten() {
        let (Some(mid), Some(point)) = (mids.get_mut(k), points.get_mut(k)) else { continue };
        if let Some(mid) = mid {
            *mid = *mid + (uv - point.0);
        }
        point.0 = uv;
    }
    snap_repeats(&mut points);
    // One snapped point per loop point, in loop order.
    for (p, snapped) in loops.iter_mut().flatten().zip(points) {
        p.0 = snapped.0;
    }
    for pts in loops {
        let first = segs.len();
        let m = pts.len();
        if m < 3 {
            out.extend(pts.iter().map(|&(_, e, k)| (e, k)));
        }
        for (i, (&(a, e, k), next)) in closed_sides(pts.iter()).enumerate() {
            segs.push((a, next.0, e, k));
            neighbours.push((first + (i + m - 1) % m, first + (i + 1) % m));
        }
    }
    let lo = |s: &(Vec2, Vec2, usize, usize)| s.0.x.min(s.1.x);
    // The segments by their least u; a stable sort, so ties keep their order.
    let mut by_lo: Vec<(usize, f64)> = segs.iter().map(lo).enumerate().collect();
    by_lo.sort_by(|x, y| x.1.total_cmp(&y.1));
    let order: Vec<usize> = by_lo.into_iter().map(|(i, _)| i).collect();
    for (oi, &i) in order.iter().enumerate() {
        let (Some(&seg_i), Some(&(before, after))) = (segs.get(i), neighbours.get(i)) else {
            continue;
        };
        let (a, c, ..) = seg_i;
        if a == c {
            continue;
        }
        let hi = a.x.max(c.x);
        for &j in order.iter().skip(oi + 1) {
            let Some(seg_j) = segs.get(j) else { continue };
            let (p, q, ..) = *seg_j;
            if lo(seg_j) > hi {
                break;
            }
            if p == q || before == j || after == j {
                continue;
            }
            if a.y.max(c.y) < p.y.min(q.y) || p.y.max(q.y) < a.y.min(c.y) {
                continue;
            }
            if [p, q].contains(&a) || [p, q].contains(&c) {
                continue;
            }
            let (o1, o2) = (orient(a, c, p), orient(a, c, q));
            let (o3, o4) = (orient(p, q, a), orient(p, q, c));
            if o1 * o2 <= 0.0 && o3 * o4 <= 0.0 {
                out.push((seg_i.2, seg_i.3));
                out.push((seg_j.2, seg_j.3));
            }
        }
    }
    let mut by_x: Vec<Vec2> = segs.iter().map(|s| s.0).collect();
    by_x.sort_by(|p, q| p.x.total_cmp(&q.x));
    for (&(a, c, e, k), &mid) in segs.iter().zip(&mids) {
        let Some(m) = mid else { continue };
        let turn = orient(a, c, m);
        if turn == 0.0 {
            continue;
        }
        let (x0, x1) = (a.x.min(c.x).min(m.x), a.x.max(c.x).max(m.x));
        let from = by_x.partition_point(|p| p.x < x0);
        let inside = by_x
            .iter()
            .skip(from)
            .take_while(|p| p.x <= x1)
            .filter(|&&p| p != a && p != c)
            .any(|&p| {
                // On the chord counts: a loop whose points all lie on one line bounds nothing.
                orient(a, c, p) * turn >= 0.0
                    && orient(c, m, p) * turn > 0.0
                    && orient(m, a, p) * turn > 0.0
            });
        if inside {
            out.push((e, k));
        }
    }
    out
}

/// Adds samples to an edge until, on each face that uses it, the surface over the parameter-plane
/// chord between neighbouring samples stays within the linear deflection of the curve. A face's
/// triangles along its boundary follow those chords, and the face cannot split them, since the
/// next face shares them.
fn fit_faces(
    b: &Brep,
    c: &Curve,
    mut ts: Vec<f64>,
    uses: &[CoedgeId],
    opt: &MeshOptions,
) -> Vec<f64> {
    for _ in 0..24 {
        let mut split = vec![false; ts.len().saturating_sub(1)];
        for &co in uses {
            let reversed = b.coedge(co).reversed;
            let mut order = ts.clone();
            if reversed {
                order.reverse();
            }
            let uvs = b.coedge_uvs(co, &order);
            let surface = &b.face(b.coedge_face(co)).surface;
            let n = ts.len() - 1;
            for (k, (uv, t)) in uvs.windows(2).zip(order.windows(2)).enumerate().take(n) {
                let (&[uv_a, uv_b], &[ta, tb]) = (uv, t) else { continue };
                let mid = (uv_a + uv_b) * 0.5;
                let on_curve = c.eval(0.5 * (ta + tb));
                // A step shorter than the deflection cannot stray by more than it; one that
                // still seems to stray is a parameter-plane jump that splitting cannot mend.
                let short = c.eval(ta).dist(c.eval(tb)) <= opt.linear;
                // A straight edge along a twisted surface's ruling meets the surface everywhere,
                // but the triangles on it can only follow the surface's turn if it is sampled.
                // The middle counts too: a loft side's normal can turn away and back between
                // two corners that face the same way.
                // Each end's normal a hair in from the end: on a cone's apex the radius is zero
                // to rounding, and the normal there takes its sign from that radius.
                let inward = |uv: Vec2| {
                    let at = uv + (mid - uv) * 1e-6;
                    surface.normal(at.x, at.y)
                };
                let (na, nm) = (inward(uv_a), surface.normal(mid.x, mid.y));
                let nb = inward(uv_b);
                let turn = na.angle(nb).max(na.angle(nm)).max(nm.angle(nb));
                if !short
                    && (surface.eval(mid.x, mid.y).dist(on_curve) > opt.linear
                        || turn > opt.angular)
                    && let Some(cut) = split.get_mut(if reversed { n - 1 - k } else { k })
                {
                    *cut = true;
                }
            }
        }
        if !split.contains(&true) {
            break;
        }
        let mut next = Vec::with_capacity(2 * ts.len());
        for (pair, &cut) in ts.windows(2).zip(&split) {
            let &[t0, t1] = pair else { continue };
            next.push(t0);
            if cut {
                next.push(0.5 * (t0 + t1));
            }
        }
        next.extend(ts.last());
        ts = next;
    }
    ts
}

/// Meshes every face, on as many threads as the machine offers, and appends them to `mesh`.
fn faces(b: &Brep, opt: &MeshOptions, samples: &[Vec<(f64, u32)>], mesh: &mut Mesh) -> Result<()> {
    let ids: Vec<FaceId> = b.face_ids().collect();
    let positions = &mesh.positions;
    let results = par_map(&ids, |&f| face::mesh_face(b, f, opt, samples, positions));
    let mut meshes = Vec::with_capacity(ids.len());
    for (r, &f) in results.into_iter().zip(&ids) {
        let named = |m: String| {
            let names: Vec<String> = b.face(f).names.iter().map(ToString::to_string).collect();
            format!("{m} (face {} {})", f.index(), names.join(";"))
        };
        meshes.push(r.map_err(|e| match e {
            Error::Numerical(m) => Error::Numerical(named(m)),
            Error::Unsupported(m) => Error::Unsupported(named(m)),
            other => other,
        })?);
    }
    split_shared_chords(b, &ids, &mut meshes);
    for (fm, &f) in meshes.iter().zip(&ids) {
        let start = mesh.triangles.len();
        let local: Vec<u32> = fm
            .points()
            .enumerate()
            .map(|(i, (g, p))| match g {
                Some(g) => g,
                None if i < BOX as usize => u32::MAX,
                None => {
                    mesh.positions.push(p);
                    (mesh.positions.len() - 1) as u32
                }
            })
            .collect();
        let reversed = b.face(f).reversed;
        // Every corner of the face's mesh has a global point.
        let global = |i: u32| local.get(i as usize).copied();
        let tris: Vec<[u32; 3]> = fm
            .triangles()
            .filter_map(|(_, [a, b, c])| Some([global(a)?, global(b)?, global(c)?]))
            .filter(|g| g[0] != g[1] && g[1] != g[2] && g[0] != g[2])
            .collect();
        for g in unfold(tris) {
            mesh.triangles
                .push(if reversed { [g[0], g[2], g[1]] } else { g });
        }
        mesh.faces.push((f, start..mesh.triangles.len()));
    }
    Ok(())
}

/// `tris` less every pair of triangles on the same three points facing opposite ways. Beside a pole,
/// whose copies along the parameter plane are one point, a point can be joined to the pole on both
/// sides of one chord: the two triangles cover nothing, and each of their edges would be used twice
/// too often.
fn unfold(tris: Vec<[u32; 3]>) -> Vec<[u32; 3]> {
    // Each triangle turned to start at its least point: a folded pair then reads a, b, c and a, c, b.
    let turned = |mut g: [u32; 3]| {
        let k = g.iter().enumerate().min_by_key(|&(_, v)| v).map_or(0, |(k, _)| k);
        g.rotate_left(k);
        g
    };
    let mut open: BTreeMap<[u32; 3], Vec<usize>> = BTreeMap::new();
    let mut folded = vec![false; tris.len()];
    for (i, &g) in tris.iter().enumerate() {
        let [a, b, c] = turned(g);
        if let Some(j) = open.get_mut(&[a, c, b]).and_then(Vec::pop) {
            for k in [i, j] {
                if let Some(f) = folded.get_mut(k) {
                    *f = true;
                }
            }
        } else {
            open.entry([a, b, c]).or_default().push(i);
        }
    }
    tris.into_iter()
        .zip(folded)
        .filter_map(|(g, f)| (!f).then_some(g))
        .collect()
}

/// An interior edge between two boundary points could be the very segment another face draws
/// between them, as a chord or as a boundary segment, which would make that mesh edge
/// non-manifold; a point of the face's own in its middle keeps each face's interior edges its own.
/// Uses are counted over triangles with three distinct points, since the copies of a pole or seam
/// point in one face draw the same segment more than once without using it more than twice.
fn split_shared_chords(b: &Brep, ids: &[FaceId], meshes: &mut [face::FaceMesh]) {
    let gid = |fm: &face::FaceMesh, v: u32| fm.gid(v);
    let key = |fm: &face::FaceMesh, a: u32, c: u32| {
        let (ga, gc) = (gid(fm, a)?, gid(fm, c)?);
        (ga != gc).then(|| (ga.min(gc), ga.max(gc)))
    };
    let distinct = |fm: &face::FaceMesh, tri: [u32; 3]| {
        sides(tri).iter().all(|&(a, c)| {
            let (ga, gc) = (gid(fm, a), gid(fm, c));
            ga.is_none() || ga != gc
        })
    };
    let mut uses: BTreeMap<(u32, u32), u32> = BTreeMap::new();
    for fm in meshes.iter() {
        for (_, tri) in fm.triangles().filter(|&(_, tri)| distinct(fm, tri)) {
            for (a, c) in sides(tri) {
                if let Some(g) = key(fm, a, c) {
                    *uses.entry(g).or_default() += 1;
                }
            }
        }
    }
    for (fm, &f) in meshes.iter_mut().zip(ids) {
        let mut todo = Vec::new();
        for (t, tri) in fm.triangles() {
            let fixed = fm.cdt.tri(t).fixed;
            for ((a, c), fixed) in sides(tri).into_iter().zip(fixed) {
                if a > c || fixed {
                    continue;
                }
                if key(fm, a, c).is_some_and(|g| uses.get(&g).is_some_and(|&n| n > 2)) {
                    todo.push((a, c));
                }
            }
        }
        let surf = &b.face(f).surface;
        for (a, c) in todo {
            if let Some((t, k)) = fm.cdt.find_edge(a, c) {
                fm.split_chord(surf, t, k);
            }
        }
    }
}

/// A triangle's three sides, each from a corner to the next.
fn sides(t: [u32; 3]) -> [(u32, u32); 3] {
    let [a, b, c] = t;
    [(a, b), (b, c), (c, a)]
}

/// `f` of each of `items`, in order, spread over the machine's threads.
fn par_map<I: Sync, T: Send>(items: &[I], f: impl Fn(&I) -> T + Sync) -> Vec<T> {
    let n = items.len();
    let threads = std::thread::available_parallelism()
        .map_or(1, |x| x.get())
        .min(n);
    if threads <= 1 {
        return items.iter().map(f).collect();
    }
    let next = AtomicUsize::new(0);
    let mut parts: Vec<(usize, T)> = std::thread::scope(|scope| {
        let workers: Vec<_> = (0..threads)
            .map(|_| {
                scope.spawn(|| {
                    let mut mine = Vec::new();
                    loop {
                        let i = next.fetch_add(1, Ordering::Relaxed);
                        let Some(item) = items.get(i) else {
                            break mine;
                        };
                        mine.push((i, f(item)));
                    }
                })
            })
            .collect();
        workers
            .into_iter()
            // A worker's panic is a bug in the mesher: it goes on up as that panic.
            .flat_map(|w| w.join().unwrap_or_else(|e| std::panic::resume_unwind(e)))
            .collect()
    });
    parts.sort_by_key(|p| p.0);
    parts.into_iter().map(|p| p.1).collect()
}

/// How a solid's part of a mesh fails to close, if it does.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct MeshReport {
    pub triangles: usize,
    pub boundary_edges: usize,
    pub nonmanifold_edges: usize,
    /// Edges more than two triangles use, as many each way: where pieces of the solid touch.
    pub shared_edges: usize,
    pub inconsistent_orientation_edges: usize,
    pub faces_without_mesh: usize,
    /// Triangles with a corner within the linear tolerance of the line through the other two.
    pub degenerate_triangles: usize,
    pub volume: f64,
}

impl MeshReport {
    /// Every edge used by as many triangles one way as the other, and every face meshed.
    pub fn watertight(&self) -> bool {
        self.boundary_edges == 0
            && self.nonmanifold_edges == 0
            && self.inconsistent_orientation_edges == 0
            && self.faces_without_mesh == 0
    }
}

/// The points at a triangle's three corners, or `None` when one is past `positions`.
fn corners(positions: &[Point3], [a, b, c]: [u32; 3]) -> Option<[Point3; 3]> {
    let at = |i: u32| positions.get(i as usize).copied();
    Some([at(a)?, at(b)?, at(c)?])
}

/// How the part of `mesh` on `solid`'s faces fails to close, if it does, with its volume.
pub fn report(b: &Brep, mesh: &Mesh, solid: SolidId) -> MeshReport {
    let faces: BTreeSet<FaceId> = b.solid_faces(solid).into_iter().collect();
    let mut r = MeshReport::default();
    let mut directed: BTreeMap<(u32, u32), usize> = BTreeMap::new();
    for (f, range) in &mesh.faces {
        if !faces.contains(f) {
            continue;
        }
        if range.is_empty() {
            r.faces_without_mesh += 1;
        }
        for t in mesh.triangles.get(range.clone()).into_iter().flatten() {
            let Some(p) = corners(&mesh.positions, *t) else { continue };
            r.triangles += 1;
            let n: Vec3 = (p[1] - p[0]).cross(p[2] - p[0]);
            let longest = p[0].dist(p[1]).max(p[1].dist(p[2])).max(p[2].dist(p[0]));
            if n.norm() <= crate::tol::LINEAR * longest {
                r.degenerate_triangles += 1;
            }
            r.volume += p[0].dot(p[1].cross(p[2])) / 6.0;
            for side in sides(*t) {
                *directed.entry(side).or_insert(0) += 1;
            }
        }
    }
    let mut seen = BTreeSet::new();
    for (&(a, c), &n_ab) in &directed {
        let k = if a < c { (a, c) } else { (c, a) };
        if !seen.insert(k) {
            continue;
        }
        let n_ba = directed.get(&(c, a)).copied().unwrap_or(0);
        match n_ab + n_ba {
            1 => r.boundary_edges += 1,
            2 if n_ab == 1 => {}
            2 => r.inconsistent_orientation_edges += 1,
            _ if n_ab == n_ba => r.shared_edges += 1,
            _ => r.nonmanifold_edges += 1,
        }
    }
    r
}
