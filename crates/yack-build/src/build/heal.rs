// yack: Making a file's geometry agree with itself within the kernel tolerance.
// Adapted from springrep@a03e8a1 src/step/heal.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from step/heal.rs to build/heal.rs (M2 T5); paths and visibility only; then to yack-build (M3 W7), paths and visibility only;
// M3 hardening (w-h3): its maps are BTreeMaps (D26; audited: looked up, filled and cleared only);
// chunk E: `Raw`'s accessors and the `entity` contract for every look-up by id and every list kept
// beside the raw lists; samples, chords and the normal equations by iterators and checked access.
//! Making a file's geometry agree with itself within the kernel tolerance.
//!
//! Exporters write each vertex, edge curve and surface separately, and many hold them together
//! only to their own, looser tolerance (often 1e-6 to 1e-2 mm). The kernel's own tolerance is
//! [`tol::LINEAR`], so where a vertex or edge misses its faces by more, the vertex
//! is moved onto every surface that meets there and the edge is refitted as the intersection of
//! its two faces' surfaces, through its vertices. Where faces miss each other, so that no vertex
//! or edge can lie on both (two tangent faces the file holds apart, a B-spline face that stops
//! short of its corner), their surfaces are moved too ([`adjust`]). Every solid records how far
//! its geometry moved, and one that would move more than [`MAX_HEAL`] is an error. Where healing
//! fails, or moves geometry further than the file says it meets, [`tolerate`] keeps the file's
//! geometry and gives its edges and vertices the local tolerances it does meet within.

use std::collections::{BTreeMap, BTreeSet};

use super::adjust::{Target, adjust};
use super::raw::{Raw, edge_range, entity, entity_mut, param_of};
use crate::{
    Error, Result,
    geom::{BSpline, Curve, IntersectionCurve, Surface, refine_onto_both},
    math::{Point3, Vec2, Vec3},
    numeric::solve_dense,
    tol,
};

/// How far healing may move a vertex, in mm, and in multiples of how far it was from meeting its
/// constraints: a vertex that has to go further has found some other meeting of its surfaces.
const MAX_MOVE: f64 = 0.5;
const MOVE_PER_GAP: f64 = 10.0;

fn fail<T>(id: u64, what: &str) -> Result<T> {
    Err(Error::InvalidInput(format!("STEP #{id}: {what}")))
}

/// The distance from `p` to `s`, projecting from `hint` when it is near.
fn surface_gap(s: &Surface, p: Point3) -> f64 {
    let uv = s.project(p);
    s.eval(uv.x, uv.y).dist(p)
}

/// Distances to a surface from points taken in order along a curve, each projected from the
/// last one's foot: [`Surface::project_near`] takes the local foot only where the surface passes
/// within its near radius, which is the global foot, so the distances are [`surface_gap`]'s
/// without a global search per point (refits measured 30% of a read in global projections).
struct Along<'a> {
    s: &'a Surface,
    hint: Option<Vec2>,
}

impl<'a> Along<'a> {
    fn new(s: &'a Surface) -> Self {
        Along { s, hint: None }
    }

    fn gap(&mut self, p: Point3) -> f64 {
        let uv = match self.hint {
            Some(h) => self.s.project_near(p, h),
            None => self.s.project(p),
        };
        self.hint = Some(uv);
        self.s.eval(uv.x, uv.y).dist(p)
    }
}

/// The even steps [`crate::check`] samples an edge at.
const CHECK_SAMPLES: usize = 16;

fn sample_params(c: &Curve, r: (f64, f64), per_span: usize) -> Vec<f64> {
    let mut cuts = vec![r.0];
    cuts.extend(c.breakpoints(r.0, r.1));
    cuts.push(r.1);
    let per = if matches!(c, Curve::Line(_)) {
        2
    } else {
        per_span.max(16 / (cuts.len() - 1))
    };
    let mut ts = Vec::new();
    for (&from, &to) in cuts.iter().zip(cuts.iter().skip(1)) {
        for k in 0..per {
            ts.push(from + (to - from) * k as f64 / per as f64);
        }
    }
    // The checker samples every edge at even steps; an edge that fits must fit there too.
    if per > 2 {
        ts.extend((1..CHECK_SAMPLES).map(|k| r.0 + (r.1 - r.0) * k as f64 / CHECK_SAMPLES as f64));
        ts.sort_by(f64::total_cmp);
    }
    ts.push(r.1);
    ts
}

/// Whether edge `e` over `range` ends at its vertices and lies on each of its faces.
fn edge_fits(raw: &Raw, e: usize, range: Option<(f64, f64)>) -> bool {
    let Some(r) = range else { return false };
    let edge = &raw.edge(e);
    let c = &edge.curve;
    if c.eval(r.0).dist(raw.point(edge.start)) > tol::LINEAR
        || c.eval(r.1).dist(raw.point(edge.end)) > tol::LINEAR
    {
        return false;
    }
    let ts = sample_params(c, r, 4);
    let mut faces = edge.faces.clone();
    faces.dedup();
    faces.iter().all(|&f| {
        let s = &raw.face(f).surface;
        // The samples start at the range's start.
        let mut hint = s.project(c.eval(ts.first().copied().unwrap_or(r.0)));
        ts.iter().all(|&t| {
            let p = c.eval(t);
            let uv = s.project_near(p, hint);
            hint = uv;
            s.eval(uv.x, uv.y).dist(p) <= tol::LINEAR
        })
    })
}

/// A constraint on a vertex: lie on a surface, on a curve, or on the plane through a point
/// square to a direction.
enum On<'a> {
    Surface(&'a Surface),
    Curve(&'a Curve),
    Plane(Point3, Vec3),
}

/// How far from the file's curve a point of a refitted edge may be found: edges whose faces meet
/// away from it are moved onto their meeting.
const MEET_REACH: f64 = 0.05;

/// The point nearest `p0` that lies on every constraint within a small part of the tolerance, by
/// damped Gauss–Newton on the distances; `None` if they do not meet within `reach` of `p0`
/// (by default, a few times the distance `p0` misses them by).
fn solve_vertex(p0: Point3, on: &[On], reach: Option<f64>) -> Option<Point3> {
    let mut x = p0;
    let mut reach = reach;
    let mut best = (f64::INFINITY, p0);
    for _ in 0..100 {
        let mut rows: Vec<(Vec3, f64)> = Vec::new();
        for c in on {
            match c {
                On::Surface(s) => {
                    let uv = s.project(x);
                    let q = s.eval(uv.x, uv.y);
                    let d = x - q;
                    let n = d
                        .unit()
                        .filter(|_| d.norm() > 1e-14)
                        .unwrap_or(s.normal(uv.x, uv.y));
                    rows.push((n, n.dot(d)));
                }
                On::Curve(c) => {
                    let t = param_of(c, x);
                    let (q, tan) = c.d1(t);
                    let d = x - q;
                    let tan = tan.unit_or_zero();
                    let e1 = (d - tan * d.dot(tan))
                        .unit()
                        .filter(|_| d.norm() > 1e-14)
                        .unwrap_or(tan.any_perpendicular().unit_or_zero());
                    let e2 = tan.cross(e1);
                    rows.push((e1, e1.dot(d)));
                    rows.push((e2, e2.dot(d)));
                }
                On::Plane(o, n) => rows.push((*n, n.dot(x - *o))),
            }
        }
        let worst = rows.iter().map(|r| r.1.abs()).fold(0.0, f64::max);
        let reach = *reach.get_or_insert((MOVE_PER_GAP * (worst + tol::LINEAR)).min(MAX_MOVE));
        if worst < best.0 {
            best = (worst, x);
        }
        if worst <= 1e-3 * tol::LINEAR {
            break;
        }
        // The normal equations, `a` row-major 3 × 3.
        let mut a = [0.0; 9];
        let mut b = [0.0; 3];
        for (n, r) in &rows {
            let n = n.to_array();
            let (rows_of_a, _) = a.as_chunks_mut::<3>();
            for ((row, bi), &ni) in rows_of_a.iter_mut().zip(b.iter_mut()).zip(&n) {
                for (aij, &nj) in row.iter_mut().zip(&n) {
                    *aij += ni * nj;
                }
                *bi -= ni * r;
            }
        }
        // Where the constraints nearly agree in direction (faces that touch), the plain step runs
        // far along their common tangent; more damping shortens it until it stays within reach.
        let trace = a[0] + a[4] + a[8];
        let mut damp = 1e-10 * trace + 1e-300;
        let step = loop {
            let mut ad = a;
            let (rows_of_ad, _) = ad.as_chunks_mut::<3>();
            for (i, row) in rows_of_ad.iter_mut().enumerate() {
                if let Some(diagonal) = row.get_mut(i) {
                    *diagonal += damp;
                }
            }
            let st = solve_dense(ad.to_vec(), b.to_vec(), 3, 1).ok()?;
            let &[sx, sy, sz] = st.as_slice() else { return None };
            let st = Vec3::new(sx, sy, sz);
            if (x + st).dist(p0) <= reach || damp > trace {
                break st;
            }
            damp *= 100.0;
        };
        if (x + step).dist(p0) > reach {
            break;
        }
        x += step;
        if step.norm() <= 1e-16 * (1.0 + x.norm()) {
            break;
        }
    }
    let x = best.1;
    let worst = on
        .iter()
        .map(|c| match c {
            On::Surface(s) => surface_gap(s, x),
            On::Curve(c) => c.eval(param_of(c, x)).dist(x),
            On::Plane(o, n) => n.dot(x - *o).abs(),
        })
        .fold(0.0, f64::max);
    (worst <= 0.25 * tol::LINEAR && x.dist(p0) <= reach.unwrap_or(0.0)).then_some(x)
}

/// The most points a refit puts through an edge before it gives up on the faces meeting along one
/// curve: halving further only finds the same band where they touch.
const MAX_REFIT_POINTS: usize = 4096;
/// The share of its gap two rounds earlier a refit's gap must at least come down to.
const STALL_GAIN: f64 = 0.5;

/// The point on both surfaces nearest `p`: by their tangent planes where they cross, by
/// alternating projection where they touch, or onto the one surface of a seam. Where neither
/// reaches them, the point on both in the plane through `p` square to `along`, the edge's
/// direction there.
fn onto_both(a: &Surface, b: &Surface, same: bool, p: Point3, along: Vec3) -> Option<Point3> {
    let ok = |x: Point3| {
        surface_gap(a, x) <= 0.25 * tol::LINEAR && surface_gap(b, x) <= 0.25 * tol::LINEAR
    };
    if same {
        let uv = a.project(p);
        return Some(a.eval(uv.x, uv.y));
    }
    if ok(p) {
        return Some(p);
    }
    if let Some(x) = refine_onto_both(a, b, p).filter(|&x| ok(x)) {
        return Some(x);
    }
    let mut x = p;
    let (mut ha, mut hb) = (a.project(x), b.project(x));
    for _ in 0..400 {
        ha = a.project_near(x, ha);
        x = a.eval(ha.x, ha.y);
        hb = b.project_near(x, hb);
        let y = b.eval(hb.x, hb.y);
        let moved = y.dist(x);
        x = y;
        if moved <= 1e-3 * tol::LINEAR {
            break;
        }
    }
    if ok(x) {
        return Some(x);
    }
    let along = along.unit()?;
    let on = [On::Surface(a), On::Surface(b), On::Plane(p, along)];
    solve_vertex(p, &on, Some(MEET_REACH))
}

fn chord_params(points: &[Point3], closed: bool) -> Vec<f64> {
    let n = points.len();
    let count = if closed { n } else { n.saturating_sub(1) };
    let mut params = vec![0.0];
    let mut total = 0.0;
    for (a, &b) in points.iter().zip(points.iter().cycle().skip(1)).take(count) {
        total += a.dist(b);
        params.push(total);
    }
    params
}

/// Edge `e` refitted onto both its faces' surfaces through its vertices, following its file
/// curve; why not, where the surfaces do not meet along it within tolerance.
fn refit(
    raw: &Raw,
    e: usize,
    range: Option<(f64, f64)>,
    samples: Option<&Vec<Point3>>,
) -> std::result::Result<(Curve, (f64, f64)), String> {
    let edge = &raw.edge(e);
    let mut faces = edge.faces.clone();
    faces.sort_unstable();
    let &[fa, fb] = faces.as_slice() else {
        return Err(format!("the edge is used by {} faces, not 2", faces.len()));
    };
    let (a, b) = (&raw.face(fa).surface, &raw.face(fb).surface);
    let same = fa == fb;
    let closed = edge.start == edge.end;
    let (ps, pe) = (raw.point(edge.start), raw.point(edge.end));
    let c = &edge.curve;
    let r = match range {
        Some(r) => r,
        None => {
            let (t0, t1) = (param_of(c, ps), param_of(c, pe));
            match c.period() {
                Some(p) if closed => (t0, t0 + p),
                Some(p) => (t0, t0 + (t1 - t0).rem_euclid(p)),
                None => (t0.min(t1), t0.max(t1)),
            }
        }
    };
    let length: f64 = {
        let ts = sample_params(c, r, 8);
        ts.iter().zip(ts.iter().skip(1)).map(|(&t0, &t1)| c.eval(t0).dist(c.eval(t1))).sum()
    };
    let n = ((length / (100.0 * tol::LINEAR)) as usize).clamp(1, 8);
    // Where the surface step placed points along the edge, they lie on both faces already: two
    // faces that touch meet anywhere across a band, and refining points onto both picks a line.
    let placed = samples.filter(|s| !s.is_empty());
    let mut pts = match placed {
        Some(s) => {
            let mut p = vec![ps];
            p.extend(s.iter().copied());
            if !closed {
                p.push(pe);
            }
            p
        }
        None => {
            let mut p = bent(c, r, n, (ps, pe));
            if closed {
                p.pop();
            }
            p
        }
    };
    if closed && pts.len() < 3 {
        return Err("the closed edge is too short to refit".into());
    }
    let kinds = format!("{} and {}", a.kind(), b.kind());
    let refine = |p: Point3, along: Vec3| onto_both(a, b, same, p, along);
    let last = pts.len() - 1;
    let count = pts.len();
    let before = pts.clone();
    for (i, p) in pts.iter_mut().enumerate() {
        if closed || (i != 0 && i != last) {
            let t = r.0 + (r.1 - r.0) * i as f64 / (if closed { count } else { last }) as f64;
            // Along the placed points, from the one before to the one after, round a closed edge.
            let neighbours = (before.get((i + 1) % count), before.get((i + count - 1) % count));
            let along = match (placed, neighbours) {
                (Some(_), (Some(&next), Some(&prev))) => next - prev,
                _ => c.d1(t).1,
            };
            *p = refine(*p, along)
                .ok_or_else(|| format!("its {kinds} faces do not meet near it"))?;
        }
    }
    // The worst gap of each round's interpolant: a refit whose gap two doublings have not at
    // least halved has stalled (faces that touch rather than cross, bench swept/ring-carried-
    // along-a-spline doubled to 4229 points for minutes), and ends with the refusal it reaches.
    let mut worst_by_round: Vec<f64> = Vec::new();
    for _ in 0..16 {
        let approx = if closed {
            BSpline::interpolate_periodic(&pts).ok()
        } else {
            BSpline::interpolate(&pts).ok()
        };
        let Some(approx) = approx else {
            return Err("its points on both faces could not be interpolated".into());
        };
        let params = chord_params(&pts, closed);
        // Each chord's parameter span, one to a pair of neighbouring points.
        let spans = || params.iter().copied().zip(params.iter().copied().skip(1));
        let (mut on_a, mut on_b) = (Along::new(a), Along::new(b));
        let err: Vec<f64> = spans()
            .map(|(p0, p1)| {
                [0.25, 0.5, 0.75]
                    .iter()
                    .map(|f| {
                        let m = approx.eval(p0 + (p1 - p0) * f);
                        on_a.gap(m).max(on_b.gap(m))
                    })
                    .fold(0.0, f64::max)
            })
            .collect();
        if err.iter().all(|&x| x <= 0.5 * tol::LINEAR) {
            let dom = approx.domain();
            // A spline that stays on both faces everywhere is the edge as it is: evaluating it
            // costs nothing, where an intersection curve refines every point onto both faces.
            let (mut on_a, mut on_b) = (Along::new(a), Along::new(b));
            let on_both = spans().all(|(p0, p1)| {
                (1..8).all(|k| {
                    let m = approx.eval(p0 + (p1 - p0) * k as f64 / 8.0);
                    on_a.gap(m).max(on_b.gap(m)) <= 0.5 * tol::LINEAR
                })
            }) && {
                let (mut on_a, mut on_b) = (Along::new(a), Along::new(b));
                (1..CHECK_SAMPLES).all(|k| {
                    let m = approx.eval(dom.0 + (dom.1 - dom.0) * k as f64 / CHECK_SAMPLES as f64);
                    on_a.gap(m).max(on_b.gap(m)) <= 0.5 * tol::LINEAR
                })
            };
            let curve = if on_both {
                Curve::BSpline(approx)
            } else {
                Curve::Intersection(Box::new(IntersectionCurve {
                    approx,
                    surfaces: [a.clone(), b.clone()],
                }))
            };
            return Ok((curve, dom));
        }
        let mut next = Vec::with_capacity(pts.len() * 2);
        // Each chord, from a point to the next round a closed edge, with its gap and span.
        let chords = pts.iter().zip(pts.iter().cycle().skip(1)).zip(err.iter().zip(spans()));
        for ((&here, &after), (&gap, (p0, p1))) in chords {
            next.push(here);
            if gap > 0.125 * tol::LINEAR {
                let m = approx.eval(0.5 * (p0 + p1));
                let along = after - here;
                let q =
                    refine(m, along).ok_or_else(|| format!("its {kinds} faces part along it"))?;
                // A midpoint that lands on its neighbours means the faces meet along no single
                // curve here: they touch, or meet the file's curve somewhere else.
                if q.dist(here) <= 4.0 * tol::LINEAR || q.dist(after) <= 4.0 * tol::LINEAR {
                    return Err(format!(
                        "its {kinds} faces meet {gap:.1e} mm off the curve through its ends"
                    ));
                }
                next.push(q);
            }
        }
        if !closed {
            next.extend(pts.last());
        }
        pts = next;
        let worst = err.iter().copied().fold(0.0, f64::max);
        let stalled = worst_by_round.len() >= 2
            && worst_by_round.get(worst_by_round.len() - 2).is_some_and(|&w| worst > STALL_GAIN * w);
        worst_by_round.push(worst);
        if pts.len() > MAX_REFIT_POINTS || stalled {
            break;
        }
    }
    Err(format!(
        "no curve through {} points stays within tolerance of its {kinds} faces, which touch \
         rather than cross",
        pts.len()
    ))
}

/// The range of edge `e` between its (moved) vertices nearest its range `old` before they moved:
/// on a periodic curve a vertex moved back past its end must not turn a short arc into a long one.
fn range_near(raw: &Raw, e: usize, old: (f64, f64)) -> (f64, f64) {
    let edge = &raw.edge(e);
    let c = &edge.curve;
    let near = |p: Point3, to: f64| {
        let t = param_of(c, p);
        match c.period() {
            Some(per) => t + ((to - t) / per).round() * per,
            None => t,
        }
    };
    let t0 = near(raw.point(edge.start), old.0);
    match c.period() {
        Some(per) if edge.start == edge.end => (t0, t0 + per),
        _ => (t0, near(raw.point(edge.end), old.1)),
    }
}

/// How far healing moved a solid's geometry: the largest movement, where, and what moved.
#[derive(Clone, Debug, PartialEq)]
pub struct Healed {
    pub moved: f64,
    pub at: Point3,
    pub what: String,
    /// The largest distance any surface moved, which bounds how far the solid's faces moved.
    pub surfaces: f64,
}

/// Each edge's parameter range, and how far healing moved the solid's geometry.
pub type Ranges = (Vec<(f64, f64)>, Option<Healed>);

/// The most healing may move any of a solid's geometry, in mm.
pub const MAX_HEAL: f64 = 0.05;

/// How many times surfaces are moved before a solid that still does not close is rejected.
const ADJUST_ROUNDS: usize = 3;

/// A vertex or edge that would not fit, and why.
enum Failure {
    Vertex(usize, String),
    Edge(usize, String),
}

fn range_of(raw: &Raw, e: usize) -> Option<(f64, f64)> {
    let edge = &raw.edge(e);
    edge_range(
        &edge.curve,
        raw.point(edge.start),
        raw.point(edge.end),
        edge.start == edge.end,
    )
}

/// Moves each vertex that misses its faces onto them, starting from `start` where given, and
/// refits each edge that still misses. Returns each edge's range, or every vertex and edge that
/// would not fit. `loose` refits the edges at a vertex that would not move too, from where it is.
fn attempt(
    raw: &mut Raw,
    start: &BTreeMap<usize, Point3>,
    samples: &BTreeMap<usize, Vec<Point3>>,
    loose: bool,
) -> std::result::Result<Vec<(f64, f64)>, Vec<Failure>> {
    let ranges: Vec<Option<(f64, f64)>> = (0..raw.edges.len()).map(|e| range_of(raw, e)).collect();
    let fits: Vec<bool> = (0..raw.edges.len())
        .map(|e| edge_fits(raw, e, *entity(&ranges, e)))
        .collect();
    let mut at_vertex: Vec<Vec<usize>> = vec![Vec::new(); raw.points.len()];
    for (e, edge) in raw.edges.iter().enumerate() {
        entity_mut(&mut at_vertex, edge.start).push(e);
        if edge.end != edge.start {
            entity_mut(&mut at_vertex, edge.end).push(e);
        }
    }
    let mut failures = Vec::new();
    let mut moved = raw.points.clone();
    let mut stuck = vec![false; raw.points.len()];
    for (v, edges) in at_vertex.iter().enumerate() {
        let p0 = start.get(&v).copied().unwrap_or(raw.point(v));
        if edges.iter().all(|&e| *entity(&fits, e)) && p0 == raw.point(v) {
            continue;
        }
        let mut faces: Vec<usize> = edges
            .iter()
            .flat_map(|&e| raw.edge(e).faces.iter().copied())
            .collect();
        faces.sort_unstable();
        faces.dedup();
        let mut on: Vec<On> = faces
            .iter()
            .map(|&f| On::Surface(&raw.face(f).surface))
            .collect();
        let surfaces_only = on.len();
        on.extend(
            edges
                .iter()
                .filter(|&&e| *entity(&fits, e))
                .map(|&e| On::Curve(&raw.edge(e).curve)),
        );
        let p =
            solve_vertex(p0, &on, None).or_else(|| {
                solve_vertex(p0, on.get(..surfaces_only).unwrap_or_default(), None)
            });
        match p {
            Some(p) => *entity_mut(&mut moved, v) = p,
            None => {
                let gap = faces
                    .iter()
                    .map(|&f| surface_gap(&raw.face(f).surface, p0))
                    .fold(0.0, f64::max);
                *entity_mut(&mut stuck, v) = true;
                failures.push(Failure::Vertex(
                    v,
                    format!(
                        "the {} faces at an end of this edge have no common point near it (the \
                         file's vertex is {gap:.1e} mm off one)",
                        faces.len()
                    ),
                ));
            }
        }
    }
    let before = std::mem::replace(&mut raw.points, moved);
    // A vertex moved without its edge's curve keeps the edge only where the curve still ends
    // close by: a face's pcurves meet there only as closely as the curves' ends, measured in
    // parameters, which on a thin cylinder is stricter than the tolerance in space.
    let ends_close = |raw: &Raw, e: usize, r: Option<(f64, f64)>| {
        let edge = &raw.edge(e);
        r.is_some_and(|r| {
            [(r.0, edge.start), (r.1, edge.end)].iter().all(|&(t, v)| {
                raw.point(v) == *entity(&before, v)
                    || edge.curve.eval(t).dist(raw.point(v)) <= 0.25 * tol::LINEAR
            })
        })
    };

    let mut out = Vec::with_capacity(raw.edges.len());
    for (e, &old) in ranges.iter().enumerate() {
        let edge = &raw.edge(e);
        if !loose && (*entity(&stuck, edge.start) || *entity(&stuck, edge.end)) {
            out.push((0.0, 0.0));
            continue;
        }
        let r = match old {
            Some(old) => Some(range_near(raw, e, old)),
            None => range_of(raw, e),
        };
        if edge_fits(raw, e, r) && ends_close(raw, e, r) {
            out.push(r.unwrap_or((0.0, 0.0)));
            continue;
        }
        let rf = match samples.get(&e) {
            Some(s) => {
                refit(raw, e, r.or(old), Some(s)).or_else(|_| refit(raw, e, r.or(old), None))
            }
            None => refit(raw, e, r.or(old), None),
        };
        match rf {
            Ok((curve, r)) => {
                raw.edge_mut(e).curve = curve;
                out.push(r);
            }
            Err(why) => {
                failures.push(Failure::Edge(e, why));
                out.push((0.0, 0.0));
            }
        }
    }
    if failures.is_empty() {
        Ok(out)
    } else {
        Err(failures)
    }
}

/// The parameter range of edge `e` between its vertices, however far they are from its curve.
fn curve_range(raw: &Raw, e: usize) -> (f64, f64) {
    if let Some(r) = range_of(raw, e) {
        return r;
    }
    let edge = &raw.edge(e);
    let c = &edge.curve;
    let (t0, t1) = (
        param_of(c, raw.point(edge.start)),
        param_of(c, raw.point(edge.end)),
    );
    match c.period() {
        Some(p) if edge.start == edge.end => (t0, t0 + p),
        Some(p) => (t0, t0 + (t1 - t0).rem_euclid(p)),
        None => (t0.min(t1), t0.max(t1)),
    }
}

/// Points along edge `e` strictly between its ends.
fn inner_samples(raw: &Raw, e: usize, count: usize) -> Vec<Point3> {
    let c = &raw.edge(e).curve;
    let r = curve_range(raw, e);
    (1..=count)
        .map(|i| c.eval(r.0 + (r.1 - r.0) * i as f64 / (count + 1) as f64))
        .collect()
}

/// `n + 1` points along curve `c` over `r`, its two ends moved to `ends` and the points between
/// moved by the blend of those two moves. A file's vertex can sit well off its edge's curve, to
/// the side where two faces touch; following the curve unbent would lead a refit, or the surface
/// step, to a second line where the faces meet beside the vertex's.
fn bent(c: &Curve, r: (f64, f64), n: usize, ends: (Point3, Point3)) -> Vec<Point3> {
    let (d0, d1) = (ends.0 - c.eval(r.0), ends.1 - c.eval(r.1));
    let mut pts: Vec<Point3> = (0..=n)
        .map(|i| {
            let s = i as f64 / n as f64;
            c.eval(r.0 + (r.1 - r.0) * s) + d0 * (1.0 - s) + d1 * s
        })
        .collect();
    if let Some(first) = pts.first_mut() {
        *first = ends.0;
    }
    if let Some(last) = pts.last_mut() {
        *last = ends.1;
    }
    pts
}

/// The points the surface step moves: every vertex and edge on `faces`, more densely along the
/// edges that would not fit.
fn targets(
    raw: &Raw,
    faces: &BTreeSet<usize>,
    failing: &BTreeSet<usize>,
    start: &BTreeMap<usize, Point3>,
) -> Vec<Target> {
    let mut out = Vec::new();
    let mut vertices = BTreeSet::new();
    let mut at_vertex: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for (e, edge) in raw.edges.iter().enumerate() {
        at_vertex.entry(edge.start).or_default().push(e);
        at_vertex.entry(edge.end).or_default().push(e);
        if !edge.faces.iter().any(|f| faces.contains(f)) {
            continue;
        }
        vertices.insert(edge.start);
        vertices.insert(edge.end);
        let mut fs = edge.faces.clone();
        fs.sort_unstable();
        fs.dedup();
        // Faces that touch meet only where the step puts a point, so a failing edge gets several
        // points per span of its curve: between them the moved surfaces could part again.
        let count = if failing.contains(&e) {
            let r = curve_range(raw, e);
            (4 * (edge.curve.breakpoints(r.0, r.1).len() + 1)).clamp(33, 96)
        } else {
            3
        };
        let at = |v: usize| start.get(&v).copied().unwrap_or(raw.point(v));
        let pts = bent(
            &edge.curve,
            curve_range(raw, e),
            count + 1,
            (at(edge.start), at(edge.end)),
        );
        // Each point between the ends, along the way from the point before it to the one after.
        let inner = pts.iter().zip(pts.iter().skip(1)).zip(pts.iter().skip(2));
        for ((&prev, &here), &next) in inner {
            out.push(Target::sample(raw, here, fs.clone(), e, next - prev));
        }
    }
    for v in vertices {
        // Every vertex taken here ends an edge, so it has its list.
        let mut fs: Vec<usize> = at_vertex
            .get(&v)
            .into_iter()
            .flatten()
            .flat_map(|&e| raw.edge(e).faces.iter().copied())
            .collect();
        fs.sort_unstable();
        fs.dedup();
        out.push(Target::vertex(
            raw,
            start.get(&v).copied().unwrap_or(raw.point(v)),
            fs,
            v,
        ));
    }
    out
}

/// The largest distance between `file`'s geometry and `healed`'s: vertices, the surfaces of
/// `faces` along their edges, and edges given new curves.
fn movement(file: &Raw, healed: &Raw, faces: &BTreeSet<usize>) -> Option<Healed> {
    let mut best: Option<Healed> = None;
    let mut surfaces: f64 = 0.0;
    let mut note = |moved: f64, at: Point3, what: String| {
        if moved > best.as_ref().map_or(0.0, |b| b.moved) {
            best = Some(Healed {
                moved,
                at,
                what,
                surfaces: 0.0,
            });
        }
    };
    for (v, (&a, &b)) in file.points.iter().zip(&healed.points).enumerate() {
        note(a.dist(b), b, format!("vertex #{}", *entity(&file.vertex_ids, v)));
    }
    for &f in faces {
        let (s0, s1) = (&file.face(f).surface, &healed.face(f).surface);
        for (e, edge) in file.edges.iter().enumerate() {
            if !edge.faces.contains(&f) {
                continue;
            }
            for p in inner_samples(file, e, 5) {
                let uv = s0.project(p);
                let a = s0.eval(uv.x, uv.y);
                let uv = s1.project_near(a, uv);
                let b = s1.eval(uv.x, uv.y);
                surfaces = surfaces.max(a.dist(b));
                note(
                    a.dist(b),
                    b,
                    format!("the surface of face #{}", file.face(f).id),
                );
            }
        }
    }
    for (e, (a, b)) in file.edges.iter().zip(&healed.edges).enumerate() {
        if a.curve == b.curve {
            continue;
        }
        for p in inner_samples(healed, e, 7) {
            let q = a.curve.eval(param_of(&a.curve, p));
            note(p.dist(q), p, format!("edge #{}", a.id));
        }
    }
    best.map(|b| Healed { surfaces, ..b })
}

/// Makes `raw` agree with itself within the kernel tolerance, and returns each edge's parameter
/// range and how far its geometry moved. A file that already agrees is left exactly as it is.
///
/// Vertices move onto their faces and edges are refitted onto theirs; where faces miss each
/// other so that cannot work, [`adjust`] moves the surfaces of the faces involved. A solid whose
/// geometry would move more than [`MAX_HEAL`] is an error naming the distance.
/// Whether every edge of `raw` ends at its vertices and lies on its faces to [`tol::LINEAR`] as
/// the file has it: then [`heal`] keeps the file as it is, at once.
pub(crate) fn fits_as_written(raw: &Raw) -> bool {
    (0..raw.edges.len()).all(|e| edge_fits(raw, e, range_of(raw, e)))
}

/// The solid `build` makes of `file` kept as written, where its edges miss their faces by more
/// than the kernel's tolerance but no more than the file's stated uncertainty `within` (a writer
/// with a looser tolerance, such as cadrs's own): each edge and vertex is given the tolerance it
/// meets within. Healing would refit every edge onto its faces and move surfaces to meet them,
/// which on a fine thread's 546 spline edges took 80 s, for a shape no different within the
/// file's own uncertainty. `None` where the file fits to the kernel's tolerance (it reads exactly,
/// as before), states no uncertainty, or misses by more.
pub fn as_written<B>(file: &Raw, within: Option<f64>, build: impl Fn(&Raw, &Tolerated) -> Option<B>) -> Option<B> {
    let w = within?;
    file.check_ids().ok()?;
    if fits_as_written(file) {
        return None;
    }
    let mut kept = file.clone();
    let t = tolerate(&mut kept, Some(w)).ok()?;
    let bound = GAP_MARGIN * w;
    if t.edges.iter().chain(&t.vertices).all(|&x| x <= bound) { build(&kept, &t) } else { None }
}

pub fn heal(raw: &mut Raw) -> Result<Ranges> {
    raw.check_ids()?;
    let ranges: Vec<Option<(f64, f64)>> = (0..raw.edges.len()).map(|e| range_of(raw, e)).collect();
    if (0..raw.edges.len()).all(|e| edge_fits(raw, e, *entity(&ranges, e))) {
        let ranges = ranges
            .into_iter()
            .map(|r| r.unwrap_or((0.0, 0.0)))
            .collect();
        return Ok((ranges, None));
    }
    let file = raw.clone();
    let mut surfaces: Vec<Surface> = file.faces.iter().map(|f| f.surface.clone()).collect();
    let mut chosen = BTreeSet::new();
    let mut failing = BTreeSet::new();
    let mut start = BTreeMap::new();
    let mut samples: BTreeMap<usize, Vec<Point3>> = BTreeMap::new();
    let mut settled = true;
    let mut failed_before = usize::MAX;
    let with_surfaces = |surfaces: &[Surface]| {
        let mut r = file.clone();
        for (f, s) in r.faces.iter_mut().zip(surfaces) {
            f.surface = s.clone();
        }
        r
    };
    for round in 0..=ADJUST_ROUNDS {
        let mut work = with_surfaces(&surfaces);
        let failures = match attempt(&mut work, &start, &samples, false) {
            Ok(ranges) => {
                let healed = movement(&file, &work, &chosen);
                if let Some(h) = healed.as_ref().filter(|h| h.moved > MAX_HEAL) {
                    return fail(
                        file.edges.first().map_or(0, |e| e.id),
                        &format!(
                            "healing would move {} {:.1e} mm, more than the {MAX_HEAL} mm allowed",
                            h.what, h.moved
                        ),
                    );
                }
                *raw = work;
                return Ok((ranges, healed));
            }
            Err(f) => f,
        };
        // The attempt moved no surface: keep its copies, whose projection seeds are worked out.
        surfaces = work.faces.into_iter().map(|f| f.surface).collect();
        let before = chosen.len();
        for f in &failures {
            match f {
                Failure::Vertex(v, _) => {
                    for edge in file.edges.iter().filter(|e| e.start == *v || e.end == *v) {
                        chosen.extend(edge.faces.iter().copied());
                    }
                }
                Failure::Edge(e, _) => {
                    failing.insert(*e);
                    chosen.extend(file.edge(*e).faces.iter().copied());
                }
            }
        }
        // Moving the surfaces again helps only where the last move closed every gap or the
        // failures reached new faces, and only from a start no worse than the last one: a move
        // that missed its targets and left more failing than before only spreads to more faces.
        let worse = round > 0 && !settled && failures.len() > failed_before;
        failed_before = failures.len();
        let stuck = round > 0 && chosen.len() == before && !settled;
        if round == ADJUST_ROUNDS || stuck || worse {
            // An attempt that fails names what failed.
            let (id, why) = match failures.first() {
                Some(Failure::Vertex(v, why)) => (
                    file.edges
                        .iter()
                        .find(|e| e.start == *v || e.end == *v)
                        .map_or(0, |e| e.id),
                    why.clone(),
                ),
                Some(Failure::Edge(e, why)) => (file.edge(*e).id, why.clone()),
                None => (file.edges.first().map_or(0, |e| e.id), "it could not be healed".into()),
            };
            return fail(id, &why);
        }
        let mut shaped = with_surfaces(&surfaces);
        let mut pts = targets(&file, &chosen, &failing, &start);
        let faces: Vec<usize> = chosen.iter().copied().collect();
        let (at, met) = adjust(&mut shaped, &faces, &mut pts);
        settled = met;
        surfaces = shaped.faces.into_iter().map(|f| f.surface).collect();
        start = at;
        samples.clear();
        for t in &pts {
            if let Some(e) = t.edge.filter(|e| failing.contains(e)) {
                samples.entry(e).or_default().push(t.at);
            }
        }
    }
    unreachable!("the last round returns")
}

/// Moves each vertex of `raw` onto its faces and refits each edge onto its faces wherever that
/// works, leaving the rest as the file has it and every surface where it is, for [`tolerate`] to
/// hold what is left: where faces meet nowhere exactly, as four faces at a corner can miss each
/// other by far less than the file's uncertainty, healing fails, but an edge elsewhere may still
/// be further off than the file promises until it is refitted. Returns how far it moved.
pub fn heal_loosely(raw: &mut Raw) -> Option<Healed> {
    // Topology that names what it lacks is none to heal.
    raw.check_ids().ok()?;
    let file = raw.clone();
    let _ = attempt(raw, &BTreeMap::new(), &BTreeMap::new(), true);
    movement(&file, raw, &BTreeSet::new())
}

/// What [`tolerate`] gives a solid it keeps as the file has it: each edge's range, and each
/// vertex's and edge's tolerance (at least [`tol::LINEAR`]).
pub struct Tolerated {
    pub ranges: Vec<(f64, f64)>,
    pub vertices: Vec<f64>,
    pub edges: Vec<f64>,
}

/// How much wider than the largest gap sampled along an edge its tolerance is: the gap between
/// samples can be larger than at them.
const GAP_MARGIN: f64 = 1.5;

/// The range of edge `e` between its vertices as the file places them, which may lie off its
/// curve; `None` where they give no stretch of it.
fn file_range(raw: &Raw, e: usize) -> Option<(f64, f64)> {
    let edge = &raw.edge(e);
    let c = &edge.curve;
    if let Some(r) = range_of(raw, e) {
        return Some(r);
    }
    let (t0, t1) = (
        param_of(c, raw.point(edge.start)),
        param_of(c, raw.point(edge.end)),
    );
    let r = match c.period() {
        Some(p) if edge.start == edge.end => (t0, t0 + p),
        Some(p) => (t0, t0 + (t1 - t0).rem_euclid(p)),
        None if edge.start == edge.end => c.domain(),
        None => (t0.min(t1), t0.max(t1)),
    };
    (r.0.is_finite() && r.1.is_finite() && r.1 > r.0).then_some(r)
}

/// The largest distance from curve `c` over `r` to surface `s`, sampled densely.
fn curve_gap(c: &Curve, r: (f64, f64), s: &Surface) -> f64 {
    let mut ts = sample_params(c, r, 8);
    ts.extend((1..64).map(|k| r.0 + (r.1 - r.0) * k as f64 / 64.0));
    ts.sort_by(f64::total_cmp);
    ts.dedup();
    // The walk starts in the middle: an edge's end may sit at a pole of the surface, where the
    // point nearest it is found only by arriving from along the edge.
    let mid = ts.len() / 2;
    let (before, after) = ts.split_at(mid);
    let Some(&t_mid) = after.first() else { return 0.0 };
    let start = s.project(c.eval(t_mid));
    let mut worst: f64 = 0.0;
    // From the middle to the end, then from the middle back to the start.
    let back: Vec<f64> = before.iter().chain(after.first()).rev().copied().collect();
    for walk in [after.to_vec(), back] {
        let mut hint = start;
        for t in walk {
            let p = c.eval(t);
            hint = s.project_near(p, hint);
            let d = s.eval(hint.x, hint.y).dist(p);
            // A projection that stalls short of the nearest point is tried again from scratch.
            let d = if d > tol::LINEAR {
                d.min(surface_gap(s, p))
            } else {
                d
            };
            worst = worst.max(d);
        }
    }
    worst
}

/// A curve for edge `e` over its file range `r` halfway between its two faces, through its
/// vertices as they are: where the faces lie apart along it rather than crossing, the file's curve
/// on one of them misses the other by the whole gap, and this one by half.
fn midway(raw: &Raw, e: usize, r: (f64, f64)) -> Option<Curve> {
    let edge = &raw.edge(e);
    let mut faces = edge.faces.clone();
    faces.sort_unstable();
    faces.dedup();
    let [fa, fb] = faces.as_slice() else {
        return None;
    };
    let (a, b) = (&raw.face(*fa).surface, &raw.face(*fb).surface);
    let closed = edge.start == edge.end;
    let mut ts = sample_params(&edge.curve, r, 8);
    ts.sort_by(f64::total_cmp);
    ts.dedup_by(|x, y| (*x - *y).abs() <= 1e-9 * (r.1 - r.0));
    if ts.len() < 2 {
        return None;
    }
    let last = ts.len() - 1;
    let mut pts: Vec<Point3> = ts
        .iter()
        .enumerate()
        .map(|(i, &t)| match i {
            0 => raw.point(edge.start),
            i if i == last => raw.point(edge.end),
            _ => {
                let mut p = edge.curve.eval(t);
                for _ in 0..50 {
                    let (ua, ub) = (a.project(p), b.project(p));
                    let (qa, qb) = (a.eval(ua.x, ua.y), b.eval(ub.x, ub.y));
                    let next = qa + (qb - qa) * 0.5;
                    let done = next.dist(p) <= 1e-12;
                    p = next;
                    if done {
                        break;
                    }
                }
                p
            }
        })
        .collect();
    let spline = if closed {
        pts.pop();
        BSpline::interpolate_periodic(&pts)
    } else {
        BSpline::interpolate(&pts)
    };
    spline.ok().map(Curve::BSpline)
}

/// The share of an edge's length that its tolerance, or that of a vertex it ends at, may reach:
/// the spheres of its two ends then cover at most half of it. Past that the ends cannot be told
/// apart, and the solid has no definite shape.
const MAX_SHARE: f64 = 0.25;

/// An edge's length over `r`, near enough to judge a tolerance against.
fn edge_length(c: &Curve, r: (f64, f64)) -> f64 {
    let at = |k: usize| c.eval(r.0 + (r.1 - r.0) * k as f64 / 32.0);
    (0..32).map(|k| at(k).dist(at(k + 1))).sum()
}

/// Keeps a solid's geometry as the file has it where healing could not make it meet within
/// [`tol::LINEAR`]: each edge and vertex that misses its faces by more gets a local tolerance that
/// holds the gap it measures, as the exporting system's own does. What the file states as its
/// uncertainty does not bound that: exporters write gaps wider than they state. An edge that
/// misses by more than `stated` takes a curve [`midway`] between its faces where that one misses
/// by less. A tolerance past [`MAX_SHARE`] of an edge it holds is an error.
pub fn tolerate(
    raw: &mut Raw,
    stated: Option<f64>,
) -> std::result::Result<Tolerated, String> {
    let mut ranges = Vec::with_capacity(raw.edges.len());
    let mut edges = Vec::with_capacity(raw.edges.len());
    let mut vertices = vec![tol::LINEAR; raw.points.len()];
    // The shortest edge ending at each vertex.
    let mut shortest = vec![f64::INFINITY; raw.points.len()];
    let widened = |gap: f64| {
        if gap <= tol::LINEAR {
            tol::LINEAR
        } else {
            GAP_MARGIN * gap
        }
    };
    for e in 0..raw.edges.len() {
        let Some(mut r) = file_range(raw, e) else {
            return Err(format!(
                "edge #{} has no stretch between its vertices",
                raw.edge(e).id
            ));
        };
        let mut faces = raw.edge(e).faces.clone();
        faces.sort_unstable();
        faces.dedup();
        let gap_of = |c: &Curve, r: (f64, f64)| {
            faces
                .iter()
                .map(|&f| curve_gap(c, r, &raw.face(f).surface))
                .fold(0.0, f64::max)
        };
        let mut gap = gap_of(&raw.edge(e).curve, r);
        if gap > stated.unwrap_or(tol::LINEAR)
            && let Some(c) = midway(raw, e, r)
        {
            let (rm, gm) = (c.domain(), gap_of(&c, c.domain()));
            if gm < gap {
                (r, gap) = (rm, gm);
                raw.edge_mut(e).curve = c;
            }
        }
        let edge = &raw.edge(e);
        let length = edge_length(&edge.curve, r);
        let t = widened(gap);
        if t > tol::LINEAR && t > MAX_SHARE * length {
            return Err(format!(
                "edge #{} misses its faces by {gap:.1e} mm, too far for an edge {length:.1e} mm long",
                edge.id
            ));
        }
        for (v, at) in [(edge.start, r.0), (edge.end, r.1)] {
            let p = raw.point(v);
            let far = faces
                .iter()
                .map(|&f| surface_gap(&raw.face(f).surface, p))
                .fold(edge.curve.eval(at).dist(p), f64::max);
            *entity_mut(&mut vertices, v) = entity(&vertices, v).max(t).max(widened(far));
            *entity_mut(&mut shortest, v) = entity(&shortest, v).min(length);
        }
        ranges.push(r);
        edges.push(t);
    }
    for (v, &t) in vertices.iter().enumerate() {
        if t > tol::LINEAR && t > MAX_SHARE * *entity(&shortest, v) {
            return Err(format!(
                "vertex #{} lies {:.1e} mm off its edges and faces, too far for an edge {:.1e} mm \
                 long that ends at it",
                *entity(&raw.vertex_ids, v),
                t / GAP_MARGIN,
                *entity(&shortest, v)
            ));
        }
    }
    Ok(Tolerated {
        ranges,
        vertices,
        edges,
    })
}
