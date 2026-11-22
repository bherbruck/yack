// yack: the history of a solid swept by cadrs (which reports none), from the moving input.
//! A swept solid's history, which cadrs's sweep of a moving solid leaves empty: each face of the
//! result `modified` from the input face whose moving copy it lies on (its first and last copies,
//! or the envelope it sweeps), or `generated` from the input edge whose moving copy it lies on (a
//! side an edge sweeps); a face found on neither, or on two, is left out (it keeps the op's own
//! name). A one-face input made every face: no geometry is asked.

use super::{Move, p3, v3};
use crate::facade::motion::Step;
use crate::native::classify::{InFace, face_polygons, in_face};
use crate::native::curve::Curve;
use crate::native::math::P2;
use crate::native::surface::Surface;
use crate::native::topo::{Body, Edge, Face};
use crate::{
    BodyId, DefaultKernel, DistanceTarget, EdgeId, FaceId, History, InputFace, Kernel, Origin,
    Result,
};
use nalgebra::{Matrix3, Point3, Rotation3, Unit, Vector3};

/// A rigid placement: `x ↦ r x + t`.
#[derive(Clone, Copy)]
pub(crate) struct Pose {
    r: Matrix3<f64>,
    t: Vector3<f64>,
}

impl Pose {
    fn identity() -> Pose {
        Pose { r: Matrix3::identity(), t: Vector3::zeros() }
    }
    /// The turn by `angle` about the line through `point` along `axis`, then the slide `along`.
    fn turn(point: [f64; 3], axis: [f64; 3], angle: f64, along: Vector3<f64>) -> Option<Pose> {
        let a = Unit::try_new(v3(axis), 0.0)?;
        let r = Rotation3::from_axis_angle(&a, angle).into_inner();
        let o = p3(point).coords;
        Some(Pose { r, t: o - r * o + along })
    }
    /// `self` after `first`.
    fn after(&self, first: &Pose) -> Pose {
        Pose { r: self.r * first.r, t: self.r * first.t + self.t }
    }
    /// Where the point placed at `q` stood before.
    fn back(&self, q: &Point3<f64>) -> Point3<f64> {
        Point3::from(self.r.transpose() * (q.coords - self.t))
    }
}

/// The placement `m` has moved the body to at `s` of its run (0 to 1); `None` for a motion this
/// doesn't follow (a climb, a carry along a curve).
pub(crate) fn move_pose(m: &Move, s: f64) -> Option<Pose> {
    match *m {
        Move::Slide { by } => Some(Pose { r: Matrix3::identity(), t: v3(by) * s }),
        Move::Turn { point, axis, angle } => Pose::turn(point, axis, angle * s, Vector3::zeros()),
        Move::Screw { point, axis, angle, rise_per_turn } => {
            let a = Unit::try_new(v3(axis), 0.0)?;
            Pose::turn(
                point,
                axis,
                angle * s,
                a.into_inner() * (rise_per_turn * angle * s / std::f64::consts::TAU),
            )
        }
        _ => None,
    }
}

/// The placement `steps`, all run at once and applied in order at each instant, have moved the
/// body to at `s` (the body drawn where it is at every step's start).
pub(crate) fn steps_pose(steps: &[Step], s: f64) -> Option<Pose> {
    let mut at = Pose::identity();
    for step in steps {
        let p = match *step {
            Step::Slide { from, to } => Pose { r: Matrix3::identity(), t: (v3(to) - v3(from)) * s },
            Step::Turn { point, axis, from, to } => {
                Pose::turn(point, axis, (to - from) * s, Vector3::zeros())?
            }
            Step::Screw { point, axis, from, to, rise_per_turn } => {
                let a = Unit::try_new(v3(axis), 0.0)?;
                Pose::turn(
                    point,
                    axis,
                    (to - from) * s,
                    a.into_inner() * (rise_per_turn * (to - from) * s / std::f64::consts::TAU),
                )?
            }
        };
        at = p.after(&at);
    }
    Some(at)
}

/// What a sweep's history may spend on trimmed distance queries: a face's (a search of its
/// trimmed domain) costs [`FACE_QUERY`], an edge's 1. Past it the searches stop and the sweep has
/// no history (its faces keep the op's own names), so a sweep that would take long to name costs
/// little more than one that names quickly.
struct Budget(std::cell::Cell<usize>);

/// How near a face's moving copy (or an edge's) passes a point of the result face it made.
pub(super) const NEAR: f64 = 10.0 * crate::native::tol::POINT;

/// The cost of one trimmed face query, in edge queries.
const FACE_QUERY: usize = 10;

/// The budget of one sweep's history, in edge queries.
const BUDGET: usize = 20_000;

impl Budget {
    /// Spends `cost`; false, and the whole budget spent, once that would pass it.
    fn spend(&self, cost: usize) -> bool {
        let left = self.0.get();
        if left < cost {
            self.0.set(0);
            return false;
        }
        self.0.set(left - cost);
        true
    }
    /// Whether the budget ran out (some search was cut short).
    fn spent(&self) -> bool {
        self.0.get() == 0
    }
}

/// The least of `d` over the run (33 even samples, then a golden-section search about the best),
/// `None` where `d` is.
fn least<T: Copy>(d: &dyn Fn(f64) -> Option<(f64, T)>) -> Option<(f64, f64, T)> {
    let n = 32;
    let mut best = {
        let (v, t) = d(0.0)?;
        (v, 0.0, t)
    };
    let mut at = 0usize;
    for i in 1..=n {
        let s = i as f64 / n as f64;
        let (v, t) = d(s)?;
        if v < best.0 {
            (best, at) = ((v, s, t), i);
        }
    }
    refine(d, at.saturating_sub(1) as f64 / n as f64, ((at + 1).min(n)) as f64 / n as f64, best)
}

/// `best` bettered by a golden-section search of `d` over `[a, b]`.
fn refine<T: Copy>(
    d: &dyn Fn(f64) -> Option<(f64, T)>,
    mut a: f64,
    mut b: f64,
    mut best: (f64, f64, T),
) -> Option<(f64, f64, T)> {
    let g = (5f64.sqrt() - 1.0) / 2.0;
    for _ in 0..24 {
        let (x, y) = (b - g * (b - a), a + g * (b - a));
        let (fx, fy) = (d(x)?, d(y)?);
        for (v, s) in [(fx, x), (fy, y)] {
            if v.0 < best.0 {
                best = (v.0, s, v.1);
            }
        }
        if fx.0 < fy.0 {
            b = y;
        } else {
            a = x;
        }
    }
    Some(best)
}

/// How near `q` comes to `target` (a face or an edge of `body`) as the body moves through
/// `pose`'s run; with the run's fraction there and the target's point nearest `q` (in the body's
/// own place).
fn nearest(
    k: &DefaultKernel,
    q: &Point3<f64>,
    target: &dyn Fn(BodyId) -> DistanceTarget,
    body: BodyId,
    pose: &dyn Fn(f64) -> Option<Pose>,
    budget: &Budget,
) -> Option<(f64, f64, Point3<f64>)> {
    let d = |s: f64| -> Option<(f64, Point3<f64>)> {
        let p = pose(s)?.back(q);
        let t = target(body);
        budget
            .spend(if matches!(t, DistanceTarget::Face(..)) { FACE_QUERY } else { 1 })
            .then_some(())?;
        k.min_distance(&DistanceTarget::Point(p), &t).ok().map(|r| (r.distance, r.b))
    };
    least(&d)
}

/// [`nearest`] for a face on a surface whose nearest point is found whole (a plane, cylinder,
/// cone, sphere or torus): the untrimmed surface, cheap, is searched first, and the trimmed
/// face asked only where the surface's copy passes within [`NEAR`] of `q` (the untrimmed
/// distance is never more than the trimmed). `None` for another surface: ask [`nearest`].
fn nearest_on_surface(
    k: &DefaultKernel,
    q: &Point3<f64>,
    surf: &Surface,
    face: FaceId,
    body: BodyId,
    pose: &dyn Fn(f64) -> Option<Pose>,
    budget: &Budget,
) -> Option<Option<(f64, f64, Point3<f64>)>> {
    if !matches!(
        surf,
        Surface::Plane { .. }
            | Surface::Cylinder { .. }
            | Surface::Cone { .. }
            | Surface::Sphere { .. }
            | Surface::Torus { .. }
    ) {
        return None;
    }
    let du = |s: f64| -> Option<(f64, Point3<f64>)> {
        let p = pose(s)?.back(q);
        let (u, v) = surf.project(&p, None);
        let at = surf.eval(u, v);
        Some(((at - p).norm(), at))
    };
    let n = 32;
    let Some(samples) = (0..=n).map(|i| du(i as f64 / n as f64)).collect::<Option<Vec<_>>>() else {
        return Some(None);
    };
    let mut best: Option<(f64, f64, Point3<f64>)> = None;
    let mut keep = |v: (f64, f64, Point3<f64>)| {
        if best.is_none_or(|b| v.0 < b.0) {
            best = Some(v);
        }
    };
    let mut far: Option<(f64, f64, Point3<f64>)> = None;
    let at = |i: usize| samples.get(i).map_or(f64::INFINITY, |x| x.0);
    for (i, &(here, foot)) in samples.iter().enumerate() {
        let low = (i == 0 || here <= at(i - 1)) && here <= at(i + 1);
        if !low {
            continue;
        }
        let s = i as f64 / n as f64;
        let Some(m) = refine(
            &du,
            i.saturating_sub(1) as f64 / n as f64,
            ((i + 1).min(n)) as f64 / n as f64,
            (here, s, foot),
        ) else {
            return Some(None);
        };
        if m.0 > NEAR {
            if far.is_none_or(|f| m.0 < f.0) {
                far = Some(m);
            }
            continue;
        }
        // The surface's copy passes through `q` here: is the face's?
        let Some(p) = pose(m.1).map(|p| p.back(q)) else { return Some(None) };
        if !budget.spend(FACE_QUERY) {
            return Some(None);
        }
        let Ok(r) = k.min_distance(&DistanceTarget::Point(p), &DistanceTarget::Face(body, face))
        else {
            continue;
        };
        keep((r.distance, m.1, r.b));
    }
    // Off the surface's copy all run long: as near as the surface comes (more than [`NEAR`]).
    Some(best.or(far))
}

/// How near `p` is to `edge` and the edge's point nearest it, found whole where its curve is a
/// line or a circle (the foot on the whole curve, held to the edge's range); `None` for another
/// curve.
fn on_edge(edge: &Edge, p: &Point3<f64>) -> Option<(f64, Point3<f64>)> {
    let [t0, t1] = edge.t;
    let foot = match &edge.curve {
        Curve::Line { o, d } => {
            let dd = d.norm_squared();
            let t = if dd > 0.0 { (p - o).dot(d) / dd } else { t0 };
            o + d * t.clamp(t0, t1)
        }
        Curve::Circle { c, x, y, r } => {
            let at = |t: f64| c + (x * t.cos() + y * t.sin()) * *r;
            let l = p - c;
            let a = t0 + (l.dot(y).atan2(l.dot(x)) - t0).rem_euclid(std::f64::consts::TAU);
            if a <= t1 {
                at(a)
            } else {
                // Off the arc: the end nearer round the circle, which is the nearer end.
                let (a0, a1) = (at(t0), at(t1));
                if (p - a0).norm() <= (p - a1).norm() { a0 } else { a1 }
            }
        }
        _ => return None,
    };
    Some(((p - foot).norm(), foot))
}

/// Up to three points of each face of `body`, each with the face's surface normal there: the
/// most interior of a grid of (u, v) over the face's own trimmed loops (their box, in the face's
/// own period: a periodic face's samples lie in its trim, not round the whole period), each
/// inside the face by more than [`NEAR`] (one on or past its edge could lie on a neighbour's
/// moving copy, and name the face wrongly). A 9 by 9 grid, then, where none of it falls inside
/// (a sliver), a 33 by 33 one; a face with no point inside either has none, and no history.
pub(super) fn domain_points(body: &Body) -> Vec<(u64, Vec<(Point3<f64>, Vector3<f64>)>)> {
    let mut out = Vec::new();
    for (f, face) in body.faces.iter().enumerate() {
        let polys = face_polygons(body, face);
        let points = [9, 33].into_iter().map(|n| interior(face, &polys, n)).find(|p| !p.is_empty());
        out.push((f as u64, points.unwrap_or_default()));
    }
    out
}

/// The three most interior points of an `n` by `n` grid of cell centres over `polys`'s box
/// (their distance to the loops in the surface's metric), each inside `face` by more than
/// [`NEAR`] and where the surface has a normal.
fn interior(face: &Face, polys: &[Vec<P2>], n: usize) -> Vec<(Point3<f64>, Vector3<f64>)> {
    let pts = || polys.iter().flatten();
    let (u0, u1) = (
        pts().map(|p| p.x).fold(f64::INFINITY, f64::min),
        pts().map(|p| p.x).fold(f64::NEG_INFINITY, f64::max),
    );
    let (v0, v1) = (
        pts().map(|p| p.y).fold(f64::INFINITY, f64::min),
        pts().map(|p| p.y).fold(f64::NEG_INFINITY, f64::max),
    );
    if !(u0 < u1 && v0 < v1) {
        return Vec::new();
    }
    let (mu, mv) = face.surf.metric();
    // The distance, in the metric, from `q` to the nearest side of the loops.
    let clear = |q: P2| -> f64 {
        let q = P2::new(q.x * mu, q.y * mv);
        let mut best = f64::INFINITY;
        for p in polys {
            for (i, a) in p.iter().enumerate() {
                let Some(b) = p.get((i + 1) % p.len()) else { continue };
                let (a, b) = (P2::new(a.x * mu, a.y * mv), P2::new(b.x * mu, b.y * mv));
                let ab = b - a;
                let l2 = ab.norm_squared();
                let t = if l2 > 0.0 { ((q - a).dot(&ab) / l2).clamp(0.0, 1.0) } else { 0.0 };
                best = best.min((q - (a + ab * t)).norm());
            }
        }
        best
    };
    let mut kept: Vec<(f64, P2)> = Vec::new();
    for i in 0..n {
        for j in 0..n {
            let uv = P2::new(
                u0 + (u1 - u0) * (i as f64 + 0.5) / n as f64,
                v0 + (v1 - v0) * (j as f64 + 0.5) / n as f64,
            );
            if in_face(polys, &face.surf, uv, NEAR) != InFace::Inside {
                continue;
            }
            let d = clear(uv);
            if d > NEAR {
                kept.push((d, uv));
            }
        }
    }
    kept.sort_by(|a, b| b.0.total_cmp(&a.0));
    kept.iter()
        .filter_map(|&(_, uv)| {
            let n = face.surf.normal(uv.x, uv.y);
            (n.norm() > 0.0).then(|| (face.surf.eval(uv.x, uv.y), n.normalize()))
        })
        .take(3)
        .collect()
}

/// The history of `out`, `body` swept through `pose` (see the module docs); empty where the
/// motion isn't followed and `body` has more than one face.
pub(crate) fn swept_history(
    k: &DefaultKernel,
    body: BodyId,
    out: BodyId,
    pose: &dyn Fn(f64) -> Option<Pose>,
) -> Result<History> {
    let input = k.get(body)?;
    let made = k.get(out)?;
    let mut h = History::default();
    if input.faces.len() == 1 {
        h.modified = (0..made.faces.len())
            .map(|f| (FaceId(f as u64), InputFace { body, face: FaceId(0) }))
            .collect();
        return Ok(h);
    }
    if pose(0.0).is_none() {
        return Ok(h);
    }
    let tol = NEAR;
    let budget = Budget(std::cell::Cell::new(BUDGET));
    let points = domain_points(made);
    let (nf, ne) = (input.faces.len(), input.edges.len());
    // The outward normal of input face `g` at its point `at`, as placed by `pose`.
    let normal_of = |g: usize, at: &Point3<f64>, pose: &Pose| -> Option<Vector3<f64>> {
        let face = input.faces.get(g)?;
        let (u, v) = face.surf.project(at, None);
        let n = face.surf.normal(u, v) * if face.sense { 1.0 } else { -1.0 };
        (n.norm() > 0.0).then(|| pose.r * n.normalize())
    };
    let square = crate::native::tol::ANGULAR_POINT;
    for (f, pts) in points {
        if budget.spent() {
            break;
        }
        if pts.is_empty() {
            continue;
        }
        // Per point, the input faces whose moving copies pass through it facing as the face does
        // there (a face's moving edge passes through the faces it sweeps across, facing aside).
        let mut on: Vec<Vec<usize>> = Vec::new();
        for (q, n) in &pts {
            let mut hit = Vec::new();
            for g in 0..nf {
                let face = FaceId(g as u64);
                let found = match input
                    .faces
                    .get(g)
                    .and_then(|x| nearest_on_surface(k, q, &x.surf, face, body, pose, &budget))
                {
                    Some(found) => found,
                    None => nearest(k, q, &|b| DistanceTarget::Face(b, face), body, pose, &budget),
                };
                let Some((d, s, at)) = found else { continue };
                // (Unsigned: `domain_points` gives the surface's own normal, not the face's
                // outward one.)
                let facing = pose(s)
                    .and_then(|p| normal_of(g, &at, &p))
                    .is_some_and(|m| m.dot(n).abs() >= 1.0 - square);
                if d <= tol && facing {
                    hit.push(g);
                }
            }
            on.push(hit);
        }
        if on.iter().all(|hit| hit.len() == 1) {
            // (A face of pieces from two input faces: the lowest, so the name is the same each
            // time.)
            let g = on.iter().filter_map(|hit| hit.first().copied()).min().unwrap_or(0);
            h.modified.push((FaceId(f), InputFace { body, face: FaceId(g as u64) }));
        } else if on.iter().all(Vec::is_empty) {
            // On no face's copy facing so: on the copy of the one edge that sweeps it.
            let edges: Vec<usize> = (0..ne)
                .filter(|&e| {
                    let edge = EdgeId(e as u64);
                    pts.iter().all(|(q, _)| {
                        let near = match input.edges.get(e).filter(|x| on_edge(x, q).is_some()) {
                            // (A line's or a circle's: found whole, no query.)
                            Some(x) => least(&|s| on_edge(x, &pose(s)?.back(q))),
                            None => {
                                let t = |b: BodyId| DistanceTarget::Edge(b, edge);
                                nearest(k, q, &t, body, pose, &budget)
                            }
                        };
                        near.is_some_and(|r| r.0 <= tol)
                    })
                })
                .collect();
            if let [e] = edges.as_slice() {
                h.generated.push((FaceId(f), Origin::FromEdge { body, edge: EdgeId(*e as u64) }));
            }
        }
    }
    // Over budget, some answers above are cut short: none rather than some wrong.
    if budget.spent() {
        return Ok(History::default());
    }
    Ok(h)
}
