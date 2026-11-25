// yack: Intersection curves of surface pairs with no closed form: a cubic B-spline through points
// Adapted from springrep@a03e8a1 src/geom/intersection.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/geom/intersection.rs to yack-geom (M3 W3). M3 hardening:
// named constants, a fit's spans as values carrying their ends instead of index arithmetic, slice
// patterns and checked access instead of indexing, and no counted `panic!`; the same arithmetic in
// the same order.
//! Intersection curves of surface pairs with no closed form: a cubic B-spline through points
//! refined onto both surfaces, with enough points that the spline stays within tolerance of both.

use std::iter::{once, successors};

use super::{BSpline, Surface, bspline::chord_params};
use crate::{
    Error, Result,
    math::{Point3, Transform, Vec2, Vec3},
    numeric::solve2,
    tol,
};

#[derive(Clone, Debug, PartialEq)]
pub struct IntersectionCurve {
    pub approx: BSpline,
    pub surfaces: [Surface; 2],
}

/// The distance from `p` to the nearest point of `s`.
pub fn distance_to(s: &Surface, p: Point3) -> f64 {
    let uv = s.project(p);
    s.eval(uv.x, uv.y).dist(p)
}

/// A point's parameters on each of two surfaces.
pub type UvPair = (Vec2, Vec2);

/// A point within this fraction of [`tol::LINEAR`] of a surface is on it, for refinement.
const SETTLED: f64 = 1e-3; // tol: fraction of the linear tolerance

/// A point within this fraction of [`tol::LINEAR`] of both surfaces is on them to begin with.
const EXACT: f64 = 1e-2;

/// The nearest a crossing search samples beside its start, before each step grows by
/// [`GRID_GROWTH`].
const GRID_START: f64 = 1e-6; // tol: the finest sampling step, in mm

/// Each sample of a crossing search is this much farther out than the one before.
const GRID_GROWTH: f64 = 1.2;

/// The halvings that pin a crossing or a least gap: far past the last bit.
const BISECTIONS: usize = 80;

/// The most Newton steps along the contour of the gap.
const CONTOUR_STEPS: usize = 40;

/// A gap below this, relative to the point's size, has settled to rounding.
const GAP_ROUNDING: f64 = 1e-14; // tol: relative rounding of a gap

/// Below this the three planes of a step held across the curve meet in no point.
const PARALLEL_PLANES: f64 = 1e-14; // tol: a unit triple product, not a length

/// The most alternating projections that place a point on both surfaces.
const REFINE_STEPS: usize = 100;

/// A refinement step below this, relative to the point's size, has stalled.
const STALLED: f64 = 1e-15; // tol: relative rounding of a step

/// The Newton steps that place the point of a span nearest a point.
const SPAN_STEPS: usize = 8;

/// The rounds of splitting a fit takes to reach the tolerance before it fails.
const FIT_ROUNDS: usize = 16;

/// The rounds of splitting a sampled fit takes to reach the tolerance before it fails.
const SAMPLED_FIT_ROUNDS: usize = 24;

/// The most samples a sampled fit takes.
const MAX_SAMPLES: usize = 1 << 14;

/// A span whose error passes this fraction of [`tol::LINEAR`] is split, and its neighbours too.
const SPLIT_ERROR: f64 = 0.125;

/// The fractions of each span where a fit's error is measured.
const PROBES: [f64; 3] = [0.25, 0.5, 0.75];

/// The point on both surfaces nearest `p`, by alternating tangent-plane steps. `None` where the
/// surfaces are tangent, so the step has no unique solution, or it does not converge.
pub fn refine_onto_both(a: &Surface, b: &Surface, p: Point3) -> Option<Point3> {
    refine(a, b, p, None, None).map(|(point, _)| point)
}

/// [`refine_onto_both`] from `hints`, the parameters of a point near `p` on each surface, and the
/// refined point's own parameters with it.
pub fn refine_onto_both_near(
    a: &Surface,
    b: &Surface,
    p: Point3,
    hints: Option<UvPair>,
) -> Option<(Point3, UvPair)> {
    refine_near(a, b, p, hints, None)
}

/// The point on both surfaces in the plane through `p` square to the unit `across`. Where a curve
/// turns sharply, the nearest point of the intersection can lie on another stretch of it; held to
/// the plane square to the curve's direction, the point stays between its neighbours.
pub fn refine_across(a: &Surface, b: &Surface, p: Point3, across: Vec3) -> Option<Point3> {
    refine(a, b, p, None, Some(across)).map(|(point, _)| point)
}

/// The points on both surfaces along the line through `x` across the curve: square to `dir` in
/// `a`'s tangent plane, out to `reach` either way, nearest first. Where the surfaces cross at a
/// tiny angle a Newton step's error is the gap's over that angle, and the steps wander; the gap
/// from `a` to `b` along the line changes sign at each crossing, and halving the bracket finds it
/// however small the angle.
fn cross_roots(
    a: &Surface,
    b: &Surface,
    x: Point3,
    dir: Vec3,
    hints: UvPair,
    reach: f64,
) -> Vec<(Point3, UvPair)> {
    let ua = a.project_near(x, hints.0);
    let line = a.normal(ua.x, ua.y).cross(dir).unit_or_zero();
    if line.norm() == 0.0 || reach.is_nan() || reach <= 0.0 {
        return Vec::new();
    }
    // The point of `a` at `s` along the line, its gap to `b` along `b`'s normal, and its
    // parameters on both.
    let at = |s: f64| {
        let ua = a.project_near(x + line * s, hints.0);
        let q = a.eval(ua.x, ua.y);
        let ub = b.project_near(q, hints.1);
        ((q - b.eval(ub.x, ub.y)).dot(b.normal(ub.x, ub.y)), q, (ua, ub))
    };
    let gap = |s: f64| at(s).0;
    // The crossing in [lo, hi], where the gap's sign at the ends differs.
    let root = |mut lo: f64, mut hi: f64| {
        let below = gap(lo) < 0.0;
        for _ in 0..BISECTIONS {
            let mid = 0.5 * (lo + hi);
            if (gap(mid) < 0.0) == below {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        0.5 * (lo + hi)
    };
    // A grid finer near `x`, a fifth wider each step, out to `reach` either way.
    let steps: Vec<f64> = successors(Some(GRID_START.min(reach)), |&s| Some(s * GRID_GROWTH))
        .take_while(|&s| s <= reach)
        .collect();
    let samples: Vec<(f64, f64)> = steps
        .iter()
        .rev()
        .map(|&s| -s)
        .chain(once(0.0))
        .chain(steps.iter().copied())
        .map(|s| (s, gap(s)))
        .collect();
    let crosses = |g0: f64, g1: f64| (g0 < 0.0) != (g1 < 0.0);
    let mut found = Vec::new();
    if let [(s0, g0), (s1, g1), ..] = *samples.as_slice()
        && crosses(g0, g1)
    {
        found.push(root(s0, s1));
    }
    for window in samples.windows(3) {
        let [(s_before, g_before), (s_here, g_here), (s_after, g_after)] = *window else {
            continue;
        };
        if crosses(g_here, g_after) {
            found.push(root(s_here, s_after));
        } else if g_here.abs() < g_before.abs() && g_here.abs() <= g_after.abs() {
            // Two crossings close together (the two sides of a thin lens) can fall between
            // samples; the gap's least magnitude between the neighbours shows them.
            let (mut lo, mut hi) = (s_before, s_after);
            let golden = 0.5 * (5f64.sqrt() - 1.0);
            for _ in 0..BISECTIONS {
                let (x1, x2) = (hi - golden * (hi - lo), lo + golden * (hi - lo));
                if gap(x1).abs() <= gap(x2).abs() {
                    hi = x2;
                } else {
                    lo = x1;
                }
            }
            let least = 0.5 * (lo + hi);
            if crosses(gap(least), g_here) {
                found.push(root(s_before, least));
                found.push(root(least, s_after));
            }
        }
    }
    let mut out: Vec<(f64, Point3, UvPair)> = found
        .into_iter()
        .map(|s| {
            let (_, q, uv) = at(s);
            (s.abs(), q, uv)
        })
        .collect();
    out.sort_by(|p, q| p.0.total_cmp(&q.0));
    out.into_iter().map(|(_, q, uv)| (q, uv)).collect()
}

/// The point of both surfaces about `step` on from the one at `uv`, found on `a`: the zero contour
/// of the gap from `a` to `b` over `a`'s parameters, reached by Newton steps along the gap's
/// gradient from the point `step` ahead. Where the surfaces are nearly tangent the nearest point
/// of both to a point ahead can lie across a thin lens; held to `a` and moved only square to the
/// contour, the step stays on its own side.
pub fn refine_on_contour(
    a: &Surface,
    b: &Surface,
    (ua, ub): UvPair,
    step: Vec3,
) -> Option<(Point3, UvPair)> {
    let (_, su, sv) = a.d1(ua.x, ua.y);
    let (du, dv) =
        solve2(su.dot(su), su.dot(sv), sv.dot(su), sv.dot(sv), su.dot(step), sv.dot(step))?;
    // Past a bounded surface's parameter range its evaluation extrapolates, which no projection
    // onto it would return.
    let inside = |uv: Vec2| match a {
        Surface::Nurbs(nurbs) => {
            let ((u0, u1), (v0, v1)) = nurbs.domain();
            (u0..=u1).contains(&uv.x) && (v0..=v1).contains(&uv.y)
        }
        _ => true,
    };
    let settled = SETTLED * tol::LINEAR;
    let mut uv = Vec2::new(ua.x + du, ua.y + dv);
    let mut hint_b = ub;
    // The gap is settled to rounding, not to the tolerance: across the contour a point is placed
    // only to the gap's error over its slope, the sine between the surfaces, which here is tiny.
    let mut best: Option<(f64, Point3, UvPair)> = None;
    for _ in 0..CONTOUR_STEPS {
        if !inside(uv) {
            break;
        }
        let (q, su, sv) = a.d1(uv.x, uv.y);
        hint_b = b.project_near(q, hint_b);
        let qb = b.eval(hint_b.x, hint_b.y);
        let nb = b.normal(hint_b.x, hint_b.y);
        let gap = (q - qb).dot(nb);
        // Off `b`'s edge its nearest point is on the boundary and the gap is not a distance.
        let on_b = q.dist(qb) <= gap.abs() + settled;
        if best.as_ref().is_some_and(|(least, _, _)| gap.abs() >= *least) {
            break;
        }
        best = on_b.then_some((gap.abs(), q, (uv, hint_b)));
        if gap.abs() <= GAP_ROUNDING * (1.0 + q.norm()) {
            break;
        }
        let gradient = Vec2::new(su.dot(nb), sv.dot(nb));
        let gradient2 = gradient.dot(gradient);
        if gradient2 <= f64::MIN_POSITIVE {
            break;
        }
        uv = Vec2::new(uv.x - gradient.x * gap / gradient2, uv.y - gradient.y * gap / gradient2);
    }
    best.filter(|&(least, _, _)| least <= settled).map(|(_, point, uvs)| (point, uvs))
}

/// [`refine`] from `hints`, falling back on a start from scratch.
fn refine_near(
    a: &Surface,
    b: &Surface,
    p: Point3,
    hints: Option<UvPair>,
    across: Option<Vec3>,
) -> Option<(Point3, UvPair)> {
    // Where surfaces nearly coincide the hints can lead the steps astray; the global start is
    // the fallback.
    refine(a, b, p, hints, across).or_else(|| hints.and_then(|_| refine(a, b, p, None, across)))
}

/// The point on both surfaces nearest `p` (or, with `across`, in the plane through `p` square to
/// it) by alternating tangent-plane steps, from `hints` when given.
fn refine(
    a: &Surface,
    b: &Surface,
    p: Point3,
    mut hints: Option<UvPair>,
    across: Option<Vec3>,
) -> Option<(Point3, UvPair)> {
    let settled = SETTLED * tol::LINEAR;
    let mut x = p;
    for _ in 0..REFINE_STEPS {
        let foot = |s: &Surface, hint: Option<Vec2>| match hint {
            Some(h) => s.project_near_at(x, h),
            None => {
                let uv = s.project(x);
                (uv, s.eval(uv.x, uv.y), s.normal(uv.x, uv.y))
            }
        };
        let (ua, qa, na) = foot(a, hints.map(|h| h.0));
        let (ub, qb, nb) = foot(b, hints.map(|h| h.1));
        hints = Some((ua, ub));
        if qa.dist(x) <= settled && qb.dist(x) <= settled {
            return Some((x, (ua, ub)));
        }
        let (ra, rb) = (na.dot(qa - x), nb.dot(qb - x));
        let step = match across {
            None => {
                let (l1, l2) = solve2(na.dot(na), na.dot(nb), nb.dot(na), nb.dot(nb), ra, rb)?;
                na * l1 + nb * l2
            }
            Some(t) => {
                let det = na.dot(nb.cross(t));
                if det.abs() <= PARALLEL_PLANES {
                    return None;
                }
                let rt = t.dot(p - x);
                (nb.cross(t) * ra + t.cross(na) * rb + na.cross(nb) * rt) * (1.0 / det)
            }
        };
        x += step;
        // A step that stalls off a surface means that surface's nearest point is on its boundary,
        // where its normal no longer points at `x`: there is no meeting there.
        if step.norm() <= STALLED * (1.0 + x.norm()) {
            let near = 1e-2 * tol::LINEAR;
            return (qa.dist(x) <= near && qb.dist(x) <= near).then_some((x, (ua, ub)));
        }
    }
    None
}

/// A point of a branch with its parameters on both surfaces, from which nearby points project.
#[derive(Clone, Copy)]
struct Node {
    p: Point3,
    uv: UvPair,
}

/// One span of a fitted spline: its parameter range and the nodes at its ends.
struct Span {
    lo: f64,
    hi: f64,
    start: Node,
    end: Node,
}

/// The fewest nodes a branch needs: three round a closed one, two along an open one.
fn fewest_nodes(closed: bool) -> usize {
    if closed { 3 } else { 2 }
}

fn too_short() -> Error {
    Error::Numerical("an intersection branch is shorter than the tolerance".into())
}

/// Points that refine onto one place are one point; a branch shorter than the tolerance has no
/// curve to fit.
fn distinct(nodes: Vec<Node>, closed: bool) -> Result<Vec<Node>> {
    let count = nodes.len();
    let mut out: Vec<Node> = Vec::with_capacity(count);
    for (i, node) in nodes.into_iter().enumerate() {
        if out.last().is_none_or(|q| q.p.dist(node.p) > tol::LINEAR) {
            out.push(node);
        } else if !closed
            && i + 1 == count
            && out.len() > 1
            && let Some(last) = out.last_mut()
        {
            // An open curve ends at its last node, where it is cut off: the point beside it goes.
            *last = node;
        }
    }
    while closed
        && let [first, .., last] = out.as_slice()
        && first.p.dist(last.p) <= tol::LINEAR
    {
        out.pop();
    }
    if out.len() < fewest_nodes(closed) {
        return Err(too_short());
    }
    Ok(out)
}

/// The nodes with each that would turn the polyline back on itself dropped. Where two surfaces
/// cross at a tiny angle θ, a point on both to within ε is placed across the curve only to within
/// ε/θ, so neighbouring points can zigzag across that band; an interpolant through them folds back
/// on itself. A traced branch never reverses between neighbouring points, so a node whose chords
/// to the nodes either side of it point against each other is scatter, and every point of the band
/// is on both surfaces.
fn forward(nodes: Vec<Node>, closed: bool) -> Result<Vec<Node>> {
    let mut out: Vec<Node> = Vec::with_capacity(nodes.len());
    for node in nodes {
        while let [.., before, last] = out.as_slice() {
            if (last.p - before.p).dot(node.p - last.p) >= 0.0 {
                break;
            }
            out.pop();
        }
        out.push(node);
    }
    distinct(out, closed)
}

/// The distance from `p` to the span `[lo, hi]` of `c`, by Newton steps from the fraction `f` of it.
fn span_distance(c: &BSpline, p: Point3, (lo, hi): (f64, f64), f: f64) -> f64 {
    let mut t = lo + (hi - lo) * f;
    for _ in 0..SPAN_STEPS {
        let [q, d1, d2] = c.eval_ders(t, 2);
        let r = q - p;
        let curvature = d1.norm2() + r.dot(d2);
        if curvature <= 0.0 {
            break;
        }
        t = (t - r.dot(d1) / curvature).clamp(lo, hi);
    }
    c.eval(t).dist(p)
}

/// Fitting is internal: a failure in it is numerical, not the caller's input.
fn numerical(e: Error) -> Error {
    match e {
        Error::InvalidInput(m) => Error::Numerical(format!("intersection curve: {m}")),
        other => other,
    }
}

/// The interpolating cubic through `points`, closed or open.
fn interpolate(points: &[Point3], closed: bool) -> Result<BSpline> {
    let spline =
        if closed { BSpline::interpolate_periodic(points) } else { BSpline::interpolate(points) };
    spline.map_err(numerical)
}

/// Each value paired with the next.
fn consecutive(values: &[f64]) -> impl Iterator<Item = (f64, f64)> + '_ {
    values.iter().copied().zip(values.iter().copied().skip(1))
}

/// The span `offset` places from span `i` of `count`: round the end of a closed curve, and none
/// past the ends of an open one.
fn neighbour(i: usize, offset: isize, count: usize, closed: bool) -> Option<usize> {
    let k = i as isize + offset;
    if closed {
        Some(k.rem_euclid(count as isize) as usize)
    } else {
        (0..count as isize).contains(&k).then_some(k as usize)
    }
}

/// Whether span `i` or a neighbour of it strays by more than [`SPLIT_ERROR`] of the tolerance.
fn near_failing(errors: &[f64], i: usize, closed: bool) -> bool {
    [-1, 0, 1].into_iter().any(|offset| {
        neighbour(i, offset, errors.len(), closed)
            .and_then(|k| errors.get(k))
            .is_some_and(|&e| e > SPLIT_ERROR * tol::LINEAR)
    })
}

impl IntersectionCurve {
    /// The curve through `points`, which lie near both surfaces in order along their
    /// intersection; `closed` joins the last back to the first.
    pub fn fit(
        points: &[Point3],
        a: Surface,
        b: Surface,
        closed: bool,
    ) -> Result<IntersectionCurve> {
        IntersectionCurve::fit_near(points, None, a, b, closed)
    }

    /// [`IntersectionCurve::fit`] with each point's parameters on the two surfaces, when known.
    pub fn fit_near(
        points: &[Point3],
        uvs: Option<&[UvPair]>,
        a: Surface,
        b: Surface,
        closed: bool,
    ) -> Result<IntersectionCurve> {
        let failed = || Error::Numerical("intersection point did not converge".into());
        let node = |p: Point3, near: Option<UvPair>| {
            let uv = match near {
                Some((ha, hb)) => (a.project_near(p, ha), b.project_near(p, hb)),
                None => (a.project(p), b.project(p)),
            };
            Node { p, uv }
        };
        let gap = |m: Point3, near: UvPair| {
            let (ua, ub) = node(m, Some(near)).uv;
            a.eval(ua.x, ua.y).dist(m).max(b.eval(ub.x, ub.y).dist(m))
        };
        // The direction the branch runs at a point.
        let heading = |n: &Node| {
            let (ua, ub) = n.uv;
            a.normal(ua.x, ua.y).cross(b.normal(ub.x, ub.y))
        };
        // Where the surfaces touch, refinement has no unique step; a point there already within
        // half the tolerance of both is as near as the fit needs.
        let on_both = |p: Point3| {
            distance_to(&a, p) <= 0.5 * tol::LINEAR && distance_to(&b, p) <= 0.5 * tol::LINEAR
        };
        // Where the surfaces lie within the tolerance of each other across a strip, the
        // intersection is anywhere in it; the point halfway between `x`'s feet on the two, settled
        // a few times, is on both when the strip reaches it.
        let between = |x: Point3, hint: Option<UvPair>| {
            let mut q = x;
            for _ in 0..4 {
                let (ua, ub) = node(q, hint).uv;
                q = a.eval(ua.x, ua.y).lerp(b.eval(ub.x, ub.y), 0.5);
            }
            on_both(q).then(|| node(q, hint))
        };
        let exact = |p: Point3| {
            distance_to(&a, p) <= EXACT * tol::LINEAR && distance_to(&b, p) <= EXACT * tol::LINEAR
        };
        // An open curve's ends are where it is cut off, at vertices it must reach: refinement slides
        // a point along the curve onto both surfaces, so an end already on both stays put.
        let last = points.len().saturating_sub(1);
        let mut nodes: Vec<Node> = points
            .iter()
            .enumerate()
            .map(|(i, &p)| {
                let hint = uvs.and_then(|uvs| uvs.get(i).copied());
                if !closed && (i == 0 || i == last) && exact(p) {
                    return Ok(node(p, hint));
                }
                refine_near(&a, &b, p, hint, None)
                    .map(|(p, uv)| Node { p, uv })
                    .or_else(|| on_both(p).then(|| node(p, hint)))
                    .or_else(|| between(p, hint))
                    .ok_or_else(failed)
            })
            .collect::<Result<_>>()?;
        for _ in 0..FIT_ROUNDS {
            nodes = forward(distinct(nodes, closed)?, closed)?;
            let points: Vec<Point3> = nodes.iter().map(|n| n.p).collect();
            let approx = interpolate(&points, closed)?;
            let params = chord_params(&points, closed).map_err(numerical)?;
            let span_count = if closed { points.len() } else { points.len() - 1 };
            let spans: Vec<Span> = consecutive(&params)
                .zip(nodes.iter().zip(nodes.iter().cycle().skip(1)))
                .take(span_count)
                .map(|((lo, hi), (&start, &end))| Span { lo, hi, start, end })
                .collect();
            let errors: Vec<f64> = spans
                .iter()
                .map(|span| {
                    PROBES
                        .iter()
                        .map(|&f| {
                            let near = if f < 0.5 { &span.start } else { &span.end };
                            gap(approx.eval(span.lo + (span.hi - span.lo) * f), near.uv)
                        })
                        .fold(0.0, f64::max)
                })
                .collect();
            if errors.iter().all(|&e| e <= 0.5 * tol::LINEAR) {
                return Ok(IntersectionCurve { approx, surfaces: [a, b] });
            }
            // Spans near failing are split with the failing ones, and so are their neighbours:
            // splitting only the worst leaves uneven spacing, which moves the interpolant's error
            // into the spans beside it, one round at a time.
            let mut split: Vec<bool> =
                (0..spans.len()).map(|i| near_failing(&errors, i, closed)).collect();
            // A span far longer than a split one beside it makes the interpolant swing across it,
            // so it is split too, until no span is over three times a split neighbour's halves.
            let lengths: Vec<f64> =
                spans.iter().map(|span| span.start.p.dist(span.end.p)).collect();
            loop {
                let grown: Vec<usize> = split
                    .iter()
                    .zip(&lengths)
                    .enumerate()
                    .filter(|&(i, (&is_split, &length))| {
                        !is_split
                            && [-1, 1].into_iter().any(|offset| {
                                neighbour(i, offset, spans.len(), closed).is_some_and(|k| {
                                    split.get(k) == Some(&true)
                                        && lengths.get(k).is_some_and(|&other| length > 1.5 * other)
                                })
                            })
                    })
                    .map(|(i, _)| i)
                    .collect();
                if grown.is_empty() {
                    break;
                }
                for i in grown {
                    if let Some(is_split) = split.get_mut(i) {
                        *is_split = true;
                    }
                }
            }
            let mut next = Vec::with_capacity(nodes.len() * 2);
            for (span, &is_split) in spans.iter().zip(&split) {
                next.push(span.start);
                if !is_split {
                    continue;
                }
                let (n0, n1) = (&span.start, &span.end);
                let [m, d, _] = approx.eval_ders(0.5 * (span.lo + span.hi), 1);
                let (h0, h1) = (heading(n0), heading(n1));
                // Where the surfaces are nearly tangent, two branches bound a thin lens between
                // them, and a point off the curve by little is near both; the other branch runs
                // the other way, so a point heading against its neighbours is on it.
                let along = |n: &Node| {
                    let h = heading(n);
                    [h0, h1].iter().all(|e| {
                        e.norm() <= tol::TANGENT || h.norm() <= tol::TANGENT || h.dot(*e) >= 0.0
                    })
                };
                let tries = [(m, Some(d)), (n0.p.lerp(n1.p, 0.5), Some(n1.p - n0.p)), (m, None)];
                // From the neighbour's parameters first; from scratch where those settle on the
                // other branch of a lens.
                let q = [Some(n0.uv), None]
                    .into_iter()
                    .flat_map(|hint| tries.iter().map(move |&(x, dir)| (x, dir, hint)))
                    .filter_map(|(x, dir, hint)| {
                        refine(&a, &b, x, hint, dir.map(|d| d.unit_or_zero()))
                    })
                    .map(|(p, uv)| Node { p, uv })
                    .find(along)
                    .or_else(|| {
                        cross_roots(&a, &b, m, d.unit_or_zero(), n0.uv, n0.p.dist(n1.p))
                            .into_iter()
                            .map(|(p, uv)| Node { p, uv })
                            .filter(|n| on_both(n.p))
                            .find(along)
                    })
                    .or_else(|| between(m, Some(n0.uv)))
                    .ok_or_else(failed)?;
                next.push(q);
            }
            if !closed {
                next.extend(nodes.last());
            }
            nodes = next;
        }
        Err(Error::Numerical("intersection curve did not reach tolerance".into()))
    }

    /// The curve through `sample(s)` for `s` over `range`, where `sample` gives points exactly on
    /// both surfaces (a closed form of the intersection): points are added at the middle parameter
    /// of every span that strays from either surface by more than half the tolerance, so nothing
    /// is refined numerically and a touching point inside the range is never lost to a step.
    pub fn fit_sampled(
        sample: &dyn Fn(f64) -> Point3,
        range: (f64, f64),
        a: Surface,
        b: Surface,
        closed: bool,
        n0: usize,
    ) -> Result<IntersectionCurve> {
        let n0 = n0.max(if closed { 3 } else { 1 });
        let at = |k: usize, n: usize| range.0 + (range.1 - range.0) * k as f64 / n as f64;
        let mut ss: Vec<f64> = (0..n0).map(|k| at(k, n0)).collect();
        if !closed {
            ss.push(range.1);
        }
        let gap = |p: Point3| distance_to(&a, p).max(distance_to(&b, p));
        for _ in 0..SAMPLED_FIT_ROUNDS {
            let mut points: Vec<Point3> = Vec::with_capacity(ss.len());
            let mut keep: Vec<f64> = Vec::with_capacity(ss.len());
            for &s in &ss {
                let p = sample(s);
                if points.last().is_none_or(|q: &Point3| q.dist(p) > tol::LINEAR) {
                    points.push(p);
                    keep.push(s);
                }
            }
            if closed {
                while let [first, .., last] = points.as_slice()
                    && first.dist(*last) <= tol::LINEAR
                {
                    points.pop();
                    keep.pop();
                }
            } else if let [_, .., last_s] = keep.as_mut_slice()
                && *last_s != range.1
            {
                // The end point merged into its neighbour: the end is the one kept.
                *last_s = range.1;
                if let Some(last_point) = points.last_mut() {
                    *last_point = sample(range.1);
                }
            }
            if points.len() < fewest_nodes(closed) {
                return Err(too_short());
            }
            ss = keep;
            if ss.len() > MAX_SAMPLES {
                break;
            }
            let approx = interpolate(&points, closed)?;
            let params = chord_params(&points, closed).map_err(numerical)?;
            let span_count = if closed { points.len() } else { points.len() - 1 };
            // Each sample paired with the next, the last of a closed curve with the range's end.
            let sample_spans = || consecutive(&ss).chain(ss.last().map(|&s| (s, range.1)));
            let errors: Vec<f64> = consecutive(&params)
                .zip(sample_spans())
                .take(span_count)
                .map(|((lo, hi), (s0, s1))| {
                    PROBES
                        .iter()
                        .map(|&f| {
                            let off = gap(approx.eval(lo + (hi - lo) * f));
                            // Where the surfaces nearly touch, a point within the tolerance of
                            // both can still lie far along the band from the curve: the closed
                            // form is the curve, so the spline must pass within it of the form.
                            let exact = sample(s0 + (s1 - s0) * f);
                            off.max(span_distance(&approx, exact, (lo, hi), f))
                        })
                        .fold(0.0, f64::max)
                })
                .collect();
            if errors.iter().all(|&e| e <= 0.5 * tol::LINEAR) {
                return Ok(IntersectionCurve { approx, surfaces: [a, b] });
            }
            // As in `fit_near`: neighbours of a failing span split with it, so spacing stays even.
            let mut next = Vec::with_capacity(2 * ss.len());
            for (i, (s0, s1)) in sample_spans().take(span_count).enumerate() {
                next.push(s0);
                if near_failing(&errors, i, closed) {
                    next.push(0.5 * (s0 + s1));
                }
            }
            if !closed {
                next.push(range.1);
            }
            ss = next;
        }
        Err(Error::Numerical("intersection curve did not reach tolerance".into()))
    }

    pub fn transformed(&self, t: &Transform) -> IntersectionCurve {
        IntersectionCurve {
            approx: self.approx.transformed(t),
            surfaces: self.surfaces.each_ref().map(|s| s.transformed(t)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        Cylinder, Surface,
        math::{Frame, Vec3},
    };

    #[test]
    fn crossing_cylinders_fit_within_tolerance() {
        let a = Surface::Cylinder(Cylinder { frame: Frame::WORLD, radius: 5.0 });
        let fx = Frame::new(Vec3::ZERO, Vec3::X, Vec3::Y).unwrap();
        let b = Surface::Cylinder(Cylinder { frame: fx, radius: 3.0 });
        // The upper loop where the Ø6 bore along x pierces the Ø10 bore along z.
        let pts: Vec<Vec3> = (0..12)
            .map(|i| {
                let t = std::f64::consts::TAU * i as f64 / 12.0;
                let (y, z) = (3.0 * t.cos(), 3.0 * t.sin());
                Vec3::new((25.0 - y * y).sqrt(), y, z)
            })
            .collect();
        // Points that refine onto one place, as marching near an end leaves, are one point.
        let mut doubled = pts.clone();
        doubled.insert(3, pts[3] + Vec3::new(0.0, 0.0, 1e-9));
        doubled.push(pts[0]);
        let c = IntersectionCurve::fit(&doubled, a.clone(), b.clone(), true).unwrap();
        let (t0, t1) = c.approx.domain();
        for i in 0..200 {
            let p = c.approx.eval(t0 + (t1 - t0) * i as f64 / 200.0);
            assert!(distance_to(&a, p) < tol::LINEAR, "{}", distance_to(&a, p));
            assert!(distance_to(&b, p) < tol::LINEAR);
        }
    }
}
