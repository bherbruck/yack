// yack: where a moving body's faces graze their motion: characteristic curves traced and fitted.
// Adapted from springrep@f2ad60e src/sweep/solid/grazing.rs (MIT OR Apache-2.0, (c) 2026 Springcraft, Inc.).
// Changes: written onto yack's kernel types: checked access in place of indexing and `expect`, the
// fit's point cache a BTreeMap, the 2 × 2 solve and small helpers as named functions; tests are new
// and check each result against a closed form.
//! Where a moving body's faces graze their motion: the characteristic curves along which the
//! velocity lies in the tangent plane, `V·n = 0` (Wang and Wang 1986). Each is found from where it
//! meets its face's boundary, traced across the face by predictor steps along it and Newton steps
//! back onto it, and fitted with a cubic B-spline, or made the exact line or circle its points lie
//! on.

use std::cell::RefCell;
use std::collections::BTreeMap;
use std::f64::consts::TAU;

use yack_base::kernel::{Error, Result};
use yack_geom::{BSpline, Circle, Curve, Line, Surface};
use yack_math::kernel::{Frame, Point3, Vec2, Vec3};
use yack_tol as tol;
use yack_topo::domain::Domain;

/// How far a fitted curve may stray from the characteristic curve.
pub(crate) const FIT: f64 = tol::LINEAR / 10.0;

/// The rate a face's surface moves outward: its outward normal dotted with the velocity.
pub(crate) struct Rate<'a> {
    pub surface: &'a Surface,
    /// 1 where the outward normal is the surface's natural normal, −1 where it is the opposite.
    pub out: f64,
    pub field: &'a dyn Fn(Point3) -> Vec3,
    /// The body's size, which sets the steps derivatives are taken over.
    pub size: f64,
}

/// A parameter step of `1e-6` of the body's size on the surface, for derivatives.
const DERIVATIVE_STEP: f64 = 1e-6;  // tol: a share of the body's size for finite differences, not a tolerance
/// Newton steps `settle` takes before it judges where it has got to.
const SETTLE_STEPS: usize = 60;
/// A root search about the start of a trace samples this many angles.
const START_SAMPLES: usize = 96;
/// Bisection steps that pin a sign change (far below any double's resolution).
const BISECTIONS: usize = 60;
/// The least cosine between successive tangents of a trace: small turns between points.
const SMOOTH: f64 = 0.995;

/// Solves the 2 × 2 system `[a b; c d] x = r`, or `None` when it is singular.
fn solve2(m: [f64; 4], r: [f64; 2]) -> Option<(f64, f64)> {
    let [a, b, c, d] = m;
    let det = a * d - b * c;
    let scale = (a.abs() + b.abs()) * (c.abs() + d.abs());
    if det.abs() <= 1e-300 || det.abs() <= 1e-14 * scale {  // tol: singularity guard of a 2 x 2 solve, not a tolerance
        return None;
    }
    Some(((r[0] * d - b * r[1]) / det, (a * r[1] - c * r[0]) / det))
}

impl Rate<'_> {
    pub(crate) fn at(&self, uv: Vec2) -> f64 {
        let p = self.surface.eval(uv.x, uv.y);
        self.out * self.surface.normal(uv.x, uv.y).dot((self.field)(p))
    }

    /// The rate's gradient in the parameters, by central differences a millionth of the body's
    /// size apart on the surface.
    fn gradient(&self, uv: Vec2) -> Vec2 {
        let (_, su, sv) = self.surface.d1(uv.x, uv.y);
        let step = |d: Vec3| DERIVATIVE_STEP * self.size / d.norm().max(1e-300);  // tol: guards a division
        let (hu, hv) = (step(su), step(sv));
        let du = (self.at(uv + Vec2::new(hu, 0.0)) - self.at(uv - Vec2::new(hu, 0.0))) / (2.0 * hu);
        let dv = (self.at(uv + Vec2::new(0.0, hv)) - self.at(uv - Vec2::new(0.0, hv))) / (2.0 * hv);
        Vec2::new(du, dv)
    }

    /// The parameter step that moves a point on the surface the least distance onto the zero
    /// contour, to first order: along the gradient taken with the surface's metric.
    fn newton(&self, uv: Vec2) -> Option<Vec2> {
        let g = self.at(uv);
        let grad = self.gradient(uv);
        let (_, su, sv) = self.surface.d1(uv.x, uv.y);
        let (e, f, h) = (su.dot(su), su.dot(sv), sv.dot(sv));
        let (a, b) = solve2([e, f, f, h], [grad.x, grad.y])?;
        let denom = grad.x * a + grad.y * b;
        if denom.abs() <= f64::MIN_POSITIVE {
            return None;
        }
        Some(Vec2::new(-g * a / denom, -g * b / denom))
    }

    /// The point of the zero contour nearest `uv`, by Newton steps.
    pub(crate) fn settle(&self, uv: Vec2) -> Option<Vec2> {
        let mut uv = uv;
        let tiny = 1e-15 * self.size;  // tol: a share of the body's size: a step this small has settled
        for _ in 0..SETTLE_STEPS {
            let step = self.newton(uv)?;
            uv = uv + step;
            let (_, su, sv) = self.surface.d1(uv.x, uv.y);
            if (su * step.x + sv * step.y).norm() <= tiny {
                return Some(uv);
            }
        }
        let (_, su, sv) = self.surface.d1(uv.x, uv.y);
        let grad = self.gradient(uv);
        let slope = (su * grad.x + sv * grad.y).norm().max(f64::MIN_POSITIVE);
        (self.at(uv).abs() / slope * (su.norm() + sv.norm()) <= 1e-3 * tol::LINEAR).then_some(uv) // tol: a thousandth of the linear tolerance: the contour is reached
    }

    /// The contour's direction at `uv` in the parameter plane, scaled to unit length on the
    /// surface, turned to agree with `prev` where given.
    fn tangent(&self, uv: Vec2, prev: Option<Vec3>) -> Option<(Vec2, Vec3)> {
        let grad = self.gradient(uv);
        let t = Vec2::new(-grad.y, grad.x);
        let (_, su, sv) = self.surface.d1(uv.x, uv.y);
        let d = su * t.x + sv * t.y;
        let len = d.norm();
        if len <= f64::MIN_POSITIVE {
            return None;
        }
        let (t, d) = (t / len, d / len);
        match prev {
            Some(p) if p.dot(d) < 0.0 => Some((-t, -d)),
            _ => Some((t, d)),
        }
    }
}

/// A traced characteristic curve: its points on the contour in order, each with its parameters.
pub(crate) struct Traced {
    pub points: Vec<Point3>,
    pub uvs: Vec<Vec2>,
}

/// Where a trace ended.
pub(crate) enum Arrival {
    /// At the root of this index among those the trace was given.
    Root(usize),
    /// Across the boundary somewhere no root was given, at this point: where the contour runs
    /// into an edge the rate vanishes all along.
    Boundary(Point3),
}

/// Whether two parameter points of `s` are within a quarter period of each other in each periodic
/// parameter: one image of a point, and not another a period away.
pub(crate) fn near_image(s: &Surface, a: Vec2, b: Vec2) -> bool {
    let d = a - b;
    s.u_period().is_none_or(|p| d.x.abs() < 0.25 * p) && s.v_period().is_none_or(|p| d.y.abs() < 0.25 * p)
}

fn failed(why: &str) -> Error {
    Error::Numerical(format!("a characteristic curve {why}"))
}

/// The first point of a trace: where the contour crosses a small circle about the start inside the
/// face, found by bisection between samples of the rate round it.
fn first_point(rate: &Rate, uv0: Vec2, inside: &dyn Fn(Vec2) -> bool) -> Result<Vec2> {
    let (_, su, sv) = rate.surface.d1(uv0.x, uv0.y);
    let r = 1e-3 * rate.size;  // tol: a share of the body's size: the radius searched round a start
    let n = START_SAMPLES;
    let angle = |k: usize| TAU * k as f64 / n as f64;
    // The parameters of the point a small step from the start along the angle `a` in the tangent
    // plane.
    let at = |a: f64| -> Vec2 {
        let (c, s) = (a.cos(), a.sin());
        let dir = (su.unit_or_zero() * c) + (su.cross(sv).cross(su).unit_or_zero() * s);
        let (du, dv) = solve2([su.dot(su), su.dot(sv), su.dot(sv), sv.dot(sv)], [su.dot(dir), sv.dot(dir)])
            .unwrap_or((0.0, 0.0));
        uv0 + Vec2::new(du, dv) * r
    };
    let gs: Vec<f64> = (0..=n).map(|k| rate.at(at(angle(k)))).collect();
    for (k, pair) in gs.windows(2).enumerate() {
        let &[g0, g1] = pair else { continue };
        let uv = if g0 == 0.0 {
            at(angle(k))
        } else if g1 != 0.0 && g0.signum() != g1.signum() {
            let (mut lo, mut hi) = (angle(k), angle(k + 1));
            for _ in 0..BISECTIONS {
                let m = 0.5 * (lo + hi);
                if rate.at(at(m)).signum() == g0.signum() {
                    lo = m;
                } else {
                    hi = m;
                }
            }
            at(0.5 * (lo + hi))
        } else {
            continue;
        };
        if inside(uv) {
            return Ok(uv);
        }
    }
    Err(failed("leaves no root into its face"))
}

/// Traces the contour from `start` into the face, until it reaches the face's boundary. `roots` are
/// the places it may end, with their parameters on this face; `from` is the one it starts at, which
/// it may not end at.
pub(crate) fn trace(rate: &Rate, dom: &Domain, roots: &[(Point3, Vec2)], from: usize) -> Result<(Traced, Arrival)> {
    let size = rate.size;
    // The trace keeps the parameters it starts with, continued, so a curve that runs round a
    // period leaves the face's window at the seam it ends on instead of reading as inside again.
    let inside = |uv: Vec2| dom.winding(uv) != 0;
    let &(p0, uv0) = roots.get(from).ok_or_else(|| failed("starts at no root"))?;
    let first = first_point(rate, uv0, &inside)?;
    let first = rate.settle(first).unwrap_or(first);
    let q0 = rate.surface.eval(first.x, first.y);
    let mut uvs = vec![uv0, first];
    let mut points = vec![p0, q0];
    let mut prev = (q0 - p0).unit();
    let mut h = 2e-3 * size;  // tol: a share of the body's size: the first step of a trace
    let h_max = size / 24.0;
    let h_min = 1e-9 * size;  // tol: a share of the body's size: the least step of a trace
    let mut length = 0.0;
    loop {
        let (Some(&uv), Some(&p)) = (uvs.last(), points.last()) else {
            return Err(failed("has no points"));
        };
        // A root within reach ahead ends the trace there. A root on a seam is two places, a period
        // apart in the parameters, and the trace ends at the one its own parameters have come to.
        let ahead = roots
            .iter()
            .enumerate()
            .filter(|&(i, &(q, quv))| {
                i != from
                    && q.dist(p) <= 1.5 * h
                    && prev.is_none_or(|d| (q - p).dot(d) > 0.5 * q.dist(p))
                    && near_image(rate.surface, quv, uv)
            })
            .min_by(|a, b| (a.1.1 - uv).norm().total_cmp(&(b.1.1 - uv).norm()));
        if let Some((i, &(q, quv))) = ahead {
            uvs.push(quv);
            points.push(q);
            return Ok((Traced { points, uvs }, Arrival::Root(i)));
        }
        let Some((t, d)) = rate.tangent(uv, prev) else {
            return Err(failed("reaches a point where it crosses another"));
        };
        let mut step = h;
        let (got, q, dq) = loop {
            if step < h_min {
                return Err(failed("does not settle onto its contour"));
            }
            let Some(got) = rate.settle(uv + t * step) else {
                step *= 0.5;
                continue;
            };
            let q = rate.surface.eval(got.x, got.y);
            let moved = q.dist(p);
            let Some((_, dq)) = rate.tangent(got, Some(d)) else {
                step *= 0.5;
                continue;
            };
            // Small turns between points, and points about the step apart.
            if dq.dot(d) < SMOOTH || moved > 1.5 * step || moved < 0.5 * step {
                step *= 0.5;
                continue;
            }
            break (got, q, dq);
        };
        if !inside(got) {
            // Out of the face: a root near the step ends it, or else the step is shortened until
            // the crossing is pinned and the contour is carried on to the boundary.
            let reach = |r: Point3| segment_distance(r, p, q);
            if let Some((i, &(r, ruv))) = roots
                .iter()
                .enumerate()
                .filter(|&(i, &(r, ruv))| {
                    i != from && reach(r) <= step.max(1e-6 * size) && near_image(rate.surface, ruv, uv)  // tol: a share of the body's size: how near a root counts as reached
                })
                .min_by(|a, b| reach(a.1.0).total_cmp(&reach(b.1.0)))
            {
                uvs.push(ruv);
                points.push(r);
                return Ok((Traced { points, uvs }, Arrival::Root(i)));
            }
            if step <= 1e-7 * size {  // tol: a share of the body's size: a step this short is at the boundary
                return Ok((Traced { points, uvs }, Arrival::Boundary(q)));
            }
            h = 0.25 * step;
            continue;
        }
        length += q.dist(p);
        if length > 200.0 * size || points.len() > 200_000 {
            return Err(failed("runs on without reaching its face's boundary"));
        }
        uvs.push(got);
        points.push(q);
        prev = Some(dq);
        h = (step * 1.5).min(h_max);
    }
}

/// The distance from `r` to the segment `pq`.
fn segment_distance(r: Point3, p: Point3, q: Point3) -> f64 {
    let d = q - p;
    let l2 = d.dot(d);
    if l2 <= f64::MIN_POSITIVE {
        return r.dist(p);
    }
    let s = ((r - p).dot(d) / l2).clamp(0.0, 1.0);
    r.dist(p + d * s)
}

/// Rounds of point-adding a fit may take before it is given up.
const FIT_ROUNDS: usize = 12;

/// The curve through a traced contour, and its parameter range: the line or circle its points lie
/// on, or a cubic B-spline through them with points added until it stays within [`FIT`] of the
/// contour.
pub(crate) fn fit(rate: &Rate, traced: &Traced) -> Result<(Curve, (f64, f64))> {
    let pts = &traced.points;
    let (Some(&a), Some(&z)) = (pts.first(), pts.last()) else {
        return Err(failed("has no points to fit"));
    };
    let exact = 1e-12 * rate.size;  // tol: a share of the body's size: points on a line or circle within rounding
    if let Some((line, len)) = Line::through(a, z)
        && pts.iter().all(|&p| (p - a).cross(line.dir).norm() <= exact)
    {
        return Ok((Curve::Line(line), (0.0, len)));
    }
    if let Some(arc) = on_circle(pts, exact) {
        return Ok(arc);
    }
    // A curve that comes round to where it started, through a seam, closes by its own period.
    let closed = a.dist(z) <= exact;
    let mut pts: Vec<Point3> = pts.clone();
    let mut uvs: Vec<Vec2> = traced.uvs.clone();
    for _ in 0..FIT_ROUNDS {
        let params = chords(&pts);
        let spline = interpolate(&pts, &params, closed)?;
        let mut added = false;
        let (mut next_pts, mut next_uvs) = (Vec::new(), Vec::new());
        for (i, ((&p, &uv), (&p_next, &uv_next))) in
            pts.iter().zip(&uvs).zip(pts.iter().skip(1).zip(uvs.iter().skip(1))).enumerate()
        {
            next_pts.push(p);
            next_uvs.push(uv);
            let (Some(&t0), Some(&t1)) = (params.get(i), params.get(i + 1)) else { continue };
            let m = spline.eval(0.5 * (t0 + t1));
            let guess = rate.surface.project_near(m, uv);
            if let Some(on) = rate.settle(guess) {
                let q = rate.surface.eval(on.x, on.y);
                if q.dist(m) > FIT {
                    next_pts.push(q);
                    next_uvs.push(on);
                    added = true;
                }
            }
            if i + 2 == pts.len() {
                next_pts.push(p_next);
                next_uvs.push(uv_next);
            }
        }
        if !added {
            let range = (params.first().copied().unwrap_or(0.0), params.last().copied().unwrap_or(0.0));
            return Ok((Curve::BSpline(spline), range));
        }
        pts = next_pts;
        uvs = next_uvs;
    }
    Err(not_fitted())
}

fn not_fitted() -> Error {
    Error::Numerical("a characteristic curve's fit did not reach its tolerance".into())
}

/// The spline through `pts` at `params`, periodic where the curve is closed (its last point is its
/// first and is left out).
fn interpolate(pts: &[Point3], params: &[f64], closed: bool) -> Result<BSpline> {
    if closed && let Some((_, open)) = pts.split_last() {
        return BSpline::interpolate_periodic_at(open, params);
    }
    BSpline::interpolate_at(pts, params)
}

/// Rounds of splitting `fit_exact` may take.
const EXACT_ROUNDS: usize = 20;
/// Spans `fit_exact` starts from.
const EXACT_SPANS: usize = 32;

/// The curve through the exact points `at(s)` for `s` over `range`, which starts and ends at
/// `ends`: the line or circle they lie on, or a cubic B-spline through them at chord-length
/// parameters, a point added in each span whose middle strays from the spline by more than [`FIT`].
pub(crate) fn fit_exact(
    at: &dyn Fn(f64) -> Point3,
    range: (f64, f64),
    ends: (Point3, Point3),
    size: f64,
) -> Result<(Curve, (f64, f64))> {
    // Each round's middles are the next round's points: each point is found once.
    let seen: RefCell<BTreeMap<u64, Point3>> = RefCell::default();
    let at = |s: f64| -> Point3 {
        if let Some(&p) = seen.borrow().get(&s.to_bits()) {
            return p;
        }
        let p = at(s);
        seen.borrow_mut().insert(s.to_bits(), p);
        p
    };
    let mut ss: Vec<f64> =
        (0..=EXACT_SPANS).map(|k| range.0 + (range.1 - range.0) * k as f64 / EXACT_SPANS as f64).collect();
    let exact = 1e-12 * size;  // tol: a share of the body's size: points on a line or circle within rounding
    let closed = ends.0.dist(ends.1) <= exact;
    for round in 0..EXACT_ROUNDS {
        let last = ss.len().saturating_sub(1);
        let pts: Vec<Point3> = ss
            .iter()
            .enumerate()
            .map(|(i, &s)| match i {
                0 => ends.0,
                i if i == last => ends.1,
                _ => at(s),
            })
            .collect();
        if round == 0 {
            if let Some((line, len)) = Line::through(ends.0, ends.1)
                && pts.iter().all(|&p| (p - ends.0).cross(line.dir).norm() <= exact)
            {
                return Ok((Curve::Line(line), (0.0, len)));
            }
            if let Some(arc) = on_circle(&pts, exact) {
                return Ok(arc);
            }
        }
        let params = chords(&pts);
        let spline = interpolate(&pts, &params, closed)?;
        // Each new point moves the whole interpolant a little, so spans near the limit are split
        // too, or the refinement creeps on one span a round.
        let errs: Vec<(f64, f64)> = ss
            .windows(2)
            .zip(params.windows(2))
            .map(|(s, t)| {
                let (&[s0, s1], &[t0, t1]) = (s, t) else { return (0.0, 0.0) };
                let mid = 0.5 * (s0 + s1);
                (mid, span_distance(&spline, at(mid), (t0, t1)))
            })
            .collect();
        let added = errs.iter().any(|e| e.1 > FIT);
        if !added {
            let ends = (params.first().copied().unwrap_or(0.0), params.last().copied().unwrap_or(0.0));
            return Ok((Curve::BSpline(spline), ends));
        }
        let mut next = Vec::with_capacity(ss.len() * 2);
        for (&s, err) in ss.iter().zip(&errs) {
            next.push(s);
            if err.1 > 0.25 * FIT {
                next.push(err.0);
            }
        }
        next.extend(ss.last());
        ss = next;
    }
    Err(not_fitted())
}

/// The distance from `p` to the span `[lo, hi]` of `c`, by Newton steps from its middle.
fn span_distance(c: &BSpline, p: Point3, (lo, hi): (f64, f64)) -> f64 {
    let mut t = 0.5 * (lo + hi);
    for _ in 0..12 {
        let [q, d1, d2] = c.eval_ders(t, 2);
        let r = q - p;
        let df = d1.norm2() + r.dot(d2);
        if df <= 0.0 {
            break;
        }
        t = (t - r.dot(d1) / df).clamp(lo, hi);
    }
    c.eval(t).dist(p)
}

/// Chord-length parameters.
fn chords(pts: &[Point3]) -> Vec<f64> {
    let mut out = vec![0.0];
    let mut s = 0.0;
    for w in pts.windows(2) {
        if let &[a, b] = w {
            s += a.dist(b);
        }
        out.push(s);
    }
    out
}

/// The arc of the circle through the points, where they all lie on one within `exact`.
fn on_circle(pts: &[Point3], exact: f64) -> Option<(Curve, (f64, f64))> {
    if pts.len() < 3 {
        return None;
    }
    let (first, last) = (*pts.first()?, *pts.last()?);
    // A closed curve's ends meet: the circle is then through its first, third and two-thirds points.
    let (a, m, z) = if first.dist(last) <= exact {
        (first, *pts.get(pts.len() / 3)?, *pts.get(2 * pts.len() / 3)?)
    } else {
        (first, *pts.get(pts.len() / 2)?, last)
    };
    let (ab, ac) = (m - a, z - a);
    let n = ab.cross(ac);
    let nn = n.dot(n);
    if nn <= f64::MIN_POSITIVE {
        return None;
    }
    let centre = a + (n.cross(ab) * ac.dot(ac) + ac.cross(n) * ab.dot(ab)) / (2.0 * nn);
    let radius = centre.dist(a);
    let normal = n.unit()?;
    if pts.iter().any(|&p| ((p - centre).norm() - radius).abs() > exact || (p - centre).dot(normal).abs() > exact) {
        return None;
    }
    let frame = Frame::new(centre, normal, first - centre).ok()?;
    // The angle each point is at, unwrapped along the points.
    let mut angle = 0.0;
    let mut prev = first - centre;
    for &p in pts.iter().skip(1) {
        let q = p - centre;
        angle += prev.cross(q).dot(normal).atan2(prev.dot(q));
        prev = q;
    }
    (angle > 0.0).then(|| (Curve::Circle(Circle { frame, radius }), (0.0, angle)))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn solve2_solves_and_refuses_singular() {
        let (x, y) = solve2([2.0, 1.0, 1.0, 3.0], [5.0, 10.0]).unwrap();
        assert!((2.0 * x + y - 5.0).abs() < 1e-12 && (x + 3.0 * y - 10.0).abs() < 1e-12);
        assert!(solve2([1.0, 2.0, 2.0, 4.0], [1.0, 1.0]).is_none());
    }

    /// `n` points of the circle of radius `r` about `c` in the plane with unit normal `k`, from
    /// angle 0 through `sweep` radians, counter-clockwise about `k`.
    fn arc(c: Vec3, k: Vec3, r: f64, sweep: f64, n: usize) -> Vec<Vec3> {
        let x = k.cross(Vec3::X).unit().unwrap_or(Vec3::Y);
        let y = k.cross(x);
        (0..n).map(|i| c + (x * (sweep * i as f64 / (n - 1) as f64).cos() + y * (sweep * i as f64 / (n - 1) as f64).sin()) * r).collect()
    }

    #[test]
    fn points_on_a_tilted_circle_make_that_circle_and_its_swept_angle() {
        let k = Vec3::new(1.0, 2.0, 2.0).unit().unwrap();
        let c = Vec3::new(1.0, -1.0, 4.0);
        let pts = arc(c, k, 3.0, 4.0, 25);
        let (curve, range) = on_circle(&pts, 1e-9).unwrap();
        let Curve::Circle(got) = curve else { panic!("not a circle") };
        assert!((got.radius - 3.0).abs() < 1e-12 && got.frame.origin.dist(c) < 1e-12);
        assert!(got.frame.z.cross(k).norm() < 1e-12);
        assert!((range.1 - 4.0).abs() < 1e-12 && range.0 == 0.0);
        // The curve at the range's end is the last point.
        assert!(curve_at_end(&got, range.1).dist(pts.last().copied().unwrap()) < 1e-12);
        // A point off the circle breaks it.
        let mut off = pts;
        if let Some(p) = off.get_mut(10) {
            *p += Vec3::new(0.0, 0.0, 1e-6);
        }
        assert!(on_circle(&off, 1e-9).is_none());
    }

    fn curve_at_end(c: &Circle, t: f64) -> Vec3 {
        Curve::Circle(c.clone()).eval(t)
    }

    #[test]
    fn a_closed_loop_of_points_is_a_whole_circle() {
        let pts = arc(Vec3::ZERO, Vec3::Z, 2.0, 2.0 * PI, 41);
        let (_, range) = on_circle(&pts, 1e-9).unwrap();
        assert!((range.1 - 2.0 * PI).abs() < 1e-9, "{range:?}");
    }

    #[test]
    fn exact_points_on_a_line_fit_as_that_line() {
        let a = Vec3::new(1.0, 1.0, 1.0);
        let b = Vec3::new(4.0, 5.0, 1.0);
        let line = |s: f64| a.lerp(b, s);
        let (curve, range) = fit_exact(&line, (0.0, 1.0), (a, b), 5.0).unwrap();
        assert!(matches!(curve, Curve::Line(_)));
        assert!((range.1 - 5.0).abs() < 1e-12);
    }

    #[test]
    fn exact_points_on_a_helix_fit_within_the_tolerance_everywhere() {
        // A turn and a half of a helix of radius 2 and pitch 3: not a line or a circle.
        let at = |s: f64| Vec3::new(2.0 * s.cos(), 2.0 * s.sin(), 0.5 * s);
        let range = (0.0, 3.0 * PI);
        let (curve, span) = fit_exact(&at, range, (at(range.0), at(range.1)), 3.0).unwrap();
        let Curve::BSpline(spline) = &curve else { panic!("not a spline") };
        // The spline is at chord-length parameters; the helix's arc length is sqrt(4 + 1/4)·s.
        let speed = (4.0f64 + 0.25).sqrt();
        // The polyline's length is the arc's less what the chords cut off: within a ten-thousandth.
        assert!((span.1 - speed * range.1).abs() < 1e-3 * span.1, "{span:?}");
        // Against the helix at independent parameters: the spline's nearest point (golden-section
        // search over a window of its chord-length parameter) is within the fit tolerance.
        for k in 0..100 {
            let s = range.1 * (k as f64 + 0.37) / 100.0;
            let want = at(s);
            let centre = s * speed;
            let (mut lo, mut hi) = ((centre - 0.5).max(span.0), (centre + 0.5).min(span.1));
            let dist = |t: f64| spline.eval(t).dist(want);
            let phi = 0.5 * (5.0f64.sqrt() - 1.0);
            for _ in 0..80 {
                let (a, b) = (hi - phi * (hi - lo), lo + phi * (hi - lo));
                if dist(a) < dist(b) {
                    hi = b;
                } else {
                    lo = a;
                }
            }
            let near = dist(0.5 * (lo + hi));
            assert!(near < 2.0 * FIT, "s = {s}: {near}");
        }
    }

    #[test]
    fn a_cylinder_slid_across_its_axis_grazes_along_a_generator() {
        // The side of a cylinder of radius 2 and height 5 about z, slid along x: the rate n·x
        // vanishes where the normal is square to x, on the generators at (0, ±2, z). The one with
        // y = +2 runs from its root on the bottom circle to its root on the top: a straight line
        // 5 long, up z.
        let b = yack_build::primitive::cylinder(&Frame::WORLD, 2.0, 5.0, "c").unwrap();
        let side = b.face_ids().find(|&f| matches!(b.face(f).surface, Surface::Cylinder(_))).unwrap();
        let face = b.face(side);
        let dom = yack_topo::face_domain(&b, side);
        let field = |_: Point3| Vec3::X;
        let out = if face.reversed { -1.0 } else { 1.0 };
        let rate = Rate { surface: &face.surface, out, field: &field, size: 5.0 };
        let u = PI / 2.0;
        let roots = [
            (Vec3::new(0.0, 2.0, 0.0), Vec2::new(u, 0.0)),
            (Vec3::new(0.0, 2.0, 5.0), Vec2::new(u, 5.0)),
        ];
        let (traced, arrival) = trace(&rate, &dom, &roots, 0).unwrap();
        assert!(matches!(arrival, Arrival::Root(1)));
        for p in &traced.points {
            assert!(p.x.abs() < 1e-9 && (p.y - 2.0).abs() < 1e-9, "{p:?}");
        }
        let (curve, range) = fit(&rate, &traced).unwrap();
        let Curve::Line(line) = curve else { panic!("not a line") };
        assert!((range.1 - 5.0).abs() < 1e-9 && line.dir.cross(Vec3::Z).norm() < 1e-9);
    }

    #[test]
    fn near_image_tells_one_image_from_another_a_period_away() {
        let cyl = Surface::Cylinder(yack_geom::Cylinder { frame: yack_math::kernel::Frame::WORLD, radius: 1.0 });
        assert!(near_image(&cyl, Vec2::new(0.1, 0.0), Vec2::new(0.2, 5.0)));
        assert!(!near_image(&cyl, Vec2::new(0.1, 0.0), Vec2::new(0.1 + TAU, 0.0)));
    }
}
