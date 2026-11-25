// yack: Non-rational B-spline curves, and cubic interpolation through point lists.
// Adapted from springrep@a03e8a1 src/geom/bspline.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/geom/bspline.rs to yack-geom (M3 W3). M3 hardening: one
// checked constructor keeps the domain, spans and their polynomials carry their own ranges and
// control points (windows, not index arithmetic), interpolation reads its points by slice
// patterns, named constants, and no counted `panic!`; the same arithmetic in the same order.
//! Non-rational B-spline curves, and cubic interpolation through point lists.

use std::iter::{repeat_n, repeat_with};
use std::sync::{Arc, OnceLock};

use crate::{
    Result,
    error::invalid,
    math::{Aabb, Transform, Vec3},
    numeric::{solve_banded, solve_cyclic},
    tol,
};

/// The highest degree a B-spline may have; the kernel interpolates with cubics.
pub(super) const MAX_DEGREE: usize = 3;
const W: usize = MAX_DEGREE + 1;

/// A span's ball is grown by this fraction of its radius, and by as much again in mm, so rounding
/// cannot leave a control point outside it.
const BALL_SLACK: f64 = 1e-12; // tol: rounding of a bounding ball

/// The equal steps each piece is sampled at when searching for a nearest point.
const NEAREST_SAMPLES: u32 = 8;

/// The most Newton steps a nearest-point search takes on one piece.
const NEWTON_STEPS: usize = 50;

/// A distance derivative this small is flat: Newton's method stops there.
const FLAT_SLOPE: f64 = 1e-300; // tol: guards a division, not a geometric tolerance

/// A Newton step this small, relative to the parameter, has converged.
const NEWTON_STEP_DONE: f64 = 1e-15; // tol: relative parameter rounding

/// A quadratic coefficient this small beside the derivative's size is no quadratic.
const NEGLIGIBLE_COEFFICIENT: f64 = 1e-14; // tol: relative rounding of polynomial coefficients

/// A non-rational B-spline. A periodic one stores its first `degree` control points again at the
/// end and its knot vector unwrapped past both ends, so evaluation is the open algorithm on the
/// wrapped parameter.
#[derive(Clone, Debug, PartialEq)]
pub struct BSpline {
    degree: usize,
    knots: Vec<f64>,
    ctrl: Vec<Vec3>,
    periodic: bool,
    /// `knots[degree]` and `knots[ctrl.len()]`: the parameter range, one period when periodic.
    domain: (f64, f64),
    spans: SpanCache,
    polys: PolyCache,
}

/// Each knot span's parameter range and a ball holding its control points, so the span (inside
/// their convex hull) lies in the ball.
#[derive(Debug)]
struct Span {
    t0: f64,
    t1: f64,
    centre: Vec3,
    radius: f64,
}

/// Built on first use; it depends only on the spline, so it never takes part in equality.
#[derive(Clone, Debug, Default)]
struct SpanCache(OnceLock<Arc<Vec<Span>>>);

impl PartialEq for SpanCache {
    fn eq(&self, _: &SpanCache) -> bool {
        true
    }
}

/// Each nonempty span's curve as polynomials in the span's local parameter, by span number:
/// evaluating one is a Horner pass with no basis recurrence. Built on first use; it depends only
/// on the spline, so it never takes part in equality.
#[derive(Clone, Debug, Default)]
struct PolyCache(OnceLock<Arc<Vec<Option<SpanPoly>>>>);

/// One nonempty span's range and its curve's coefficients, lowest order first, about its start
/// and about its end.
#[derive(Debug)]
struct SpanPoly {
    t0: f64,
    t1: f64,
    about_start: Vec<Vec3>,
    about_end: Vec<Vec3>,
}

impl PartialEq for PolyCache {
    fn eq(&self, _: &PolyCache) -> bool {
        true
    }
}

/// Each knot paired with the next.
fn knot_pairs(knots: &[f64]) -> impl Iterator<Item = (f64, f64)> + '_ {
    knots.iter().copied().zip(knots.iter().copied().skip(1))
}

/// The points whose coordinates `coordinates` lists in threes.
fn points_from(coordinates: &[f64]) -> Vec<Vec3> {
    let (triples, _) = coordinates.as_chunks::<3>();
    triples.iter().map(|&[x, y, z]| Vec3::new(x, y, z)).collect()
}

impl BSpline {
    /// An open spline from its parts. The knots must be nondecreasing and number
    /// `ctrl.len() + degree + 1`.
    pub fn new(degree: usize, knots: Vec<f64>, ctrl: Vec<Vec3>) -> Result<BSpline> {
        if degree == 0 || degree > MAX_DEGREE {
            return invalid("B-spline degree must be 1 to 3");
        }
        if ctrl.len() <= degree {
            return invalid("a B-spline needs more control points than its degree");
        }
        if knots.len() != ctrl.len() + degree + 1 {
            return invalid("B-spline knot count must be control points + degree + 1");
        }
        if knot_pairs(&knots).any(|(a, b)| b < a) {
            return invalid("B-spline knots must not decrease");
        }
        let s = BSpline::from_parts(degree, knots, ctrl, false)?;
        let (a, b) = s.domain();
        if b - a <= 0.0 {
            return invalid("B-spline has an empty domain");
        }
        Ok(s)
    }

    /// The spline of parts whose counts fit, its domain read once from its knots.
    fn from_parts(
        degree: usize,
        knots: Vec<f64>,
        ctrl: Vec<Vec3>,
        periodic: bool,
    ) -> Result<BSpline> {
        let (Some(&start), Some(&end)) = (knots.get(degree), knots.get(ctrl.len())) else {
            return invalid("B-spline knot count must be control points + degree + 1");
        };
        Ok(BSpline {
            degree,
            knots,
            ctrl,
            periodic,
            domain: (start, end),
            spans: SpanCache::default(),
            polys: PolyCache::default(),
        })
    }

    pub fn degree(&self) -> usize {
        self.degree
    }

    pub fn knots(&self) -> &[f64] {
        &self.knots
    }

    pub fn control_points(&self) -> &[Vec3] {
        &self.ctrl
    }

    pub fn is_periodic(&self) -> bool {
        self.periodic
    }

    pub fn domain(&self) -> (f64, f64) {
        self.domain
    }

    pub fn period(&self) -> Option<f64> {
        self.periodic.then(|| {
            let (a, b) = self.domain();
            b - a
        })
    }

    /// The distinct knots strictly inside (t0, t1), where the curve may lose smoothness.
    pub fn breakpoints(&self, t0: f64, t1: f64) -> Vec<f64> {
        let (a, b) = self.domain();
        let mut out = Vec::new();
        let mut push = |k: f64| {
            if k > t0 && k < t1 && out.last().is_none_or(|&l: &f64| k > l) {
                out.push(k);
            }
        };
        match self.period() {
            Some(p) => {
                // The knots of one period, `knots[degree..=ctrl.len()]`, repeated.
                let period_knots =
                    self.knots.iter().skip(self.degree).take(self.ctrl.len() + 1 - self.degree);
                let start = ((t0 - a) / p).floor() as i64;
                let end = ((t1 - a) / p).ceil() as i64;
                for w in start..=end {
                    for &k in period_knots.clone() {
                        push(k + w as f64 * p);
                    }
                }
            }
            None => {
                for &k in &self.knots {
                    if k > a && k < b {
                        push(k);
                    }
                }
            }
        }
        out
    }

    fn wrap(&self, t: f64) -> f64 {
        let (a, b) = self.domain();
        match self.period() {
            Some(p) => {
                let w = a + (t - a).rem_euclid(p);
                if w >= b { a } else { w }
            }
            None => t,
        }
    }

    /// The nonempty knot span `i` (`knots[i] ≤ t < knots[i + 1]`, `degree ≤ i < ctrl.len()`) that
    /// holds `t`; past an end, the end's span.
    fn span(&self, t: f64) -> usize {
        let (p, m) = (self.degree, self.ctrl.len());
        let (start, end) = self.domain();
        let knot = |i: usize| self.knots.get(i).copied();
        if t >= end {
            let mut i = m - 1;
            while i > p && knot(i).is_some_and(|k| k >= end) {
                i -= 1;
            }
            return i;
        }
        if t <= start {
            let mut i = p;
            while i + 1 < m && knot(i + 1).is_some_and(|k| k <= start) {
                i += 1;
            }
            return i;
        }
        let (mut lo, mut hi) = (p, m);
        while hi - lo > 1 {
            let mid = (lo + hi) / 2;
            if knot(mid).is_some_and(|k| t < k) { hi = mid } else { lo = mid }
        }
        lo
    }

    /// Basis function values and derivatives up to `nd` at `t` in span `i`.
    fn basis_ders(&self, i: usize, t: f64, nd: usize) -> [[f64; W]; W] {
        super::nurbs::basis_derivatives(self.degree, &self.knots, i, t, nd)
    }

    /// The point and its first `nd` derivatives at `t`, from the span's polynomial about the
    /// nearer end of the span: exact at both ends, and its local parameter at most ½.
    pub fn eval_ders(&self, t: f64, nd: usize) -> [Vec3; 3] {
        let t = self.wrap(t);
        debug_assert!(
            matches!(self.polys().get(self.span(t)), Some(Some(_))),
            "every span `span` gives is nonempty"
        );
        let Some(Some(poly)) = self.polys().get(self.span(t)) else {
            // Every span `span` gives is nonempty, so it has its polynomials.
            return self.eval_ders_by_basis(t, nd);
        };
        let h = poly.t1 - poly.t0;
        let x = (t - poly.t0) / h;
        let (s, coefficients) =
            if x > 0.5 { (x - 1.0, &poly.about_end) } else { (x, &poly.about_start) };
        let mut highest_first = coefficients.iter().rev();
        let mut p0 = highest_first.next().copied().unwrap_or(Vec3::ZERO);
        let (mut p1, mut p2) = (Vec3::ZERO, Vec3::ZERO);
        for &c in highest_first {
            if nd >= 2 {
                p2 = p2 * s + p1;
            }
            if nd >= 1 {
                p1 = p1 * s + p0;
            }
            p0 = p0 * s + c;
        }
        [p0, p1 / h, p2 * (2.0 / (h * h))]
    }

    fn polys(&self) -> &[Option<SpanPoly>] {
        self.polys.0.get_or_init(|| {
            let p = self.degree;
            let spans = knot_pairs(&self.knots).skip(p).zip(self.ctrl.windows(p + 1)).enumerate();
            let polys = spans.map(|(k, ((t0, t1), points))| {
                if t1 <= t0 {
                    return None;
                }
                // The span's coefficient of each order: its control points weighted by their
                // basis functions' coefficients of that order.
                let series = |end: bool| {
                    let basis = super::nurbs::span_poly(p, &self.knots, k + p, end);
                    let mut coefficients = vec![Vec3::ZERO; p + 1];
                    for (&point, row) in points.iter().zip(&basis) {
                        for (c, &b) in coefficients.iter_mut().zip(row) {
                            *c += point * b;
                        }
                    }
                    coefficients
                };
                Some(SpanPoly { t0, t1, about_start: series(false), about_end: series(true) })
            });
            // The spans before the degree's are not the curve's.
            Arc::new(repeat_with(|| None).take(p).chain(polys).collect())
        })
    }

    /// [`BSpline::eval_ders`] by the basis recurrence, for checking the polynomials.
    fn eval_ders_by_basis(&self, t: f64, nd: usize) -> [Vec3; 3] {
        let t = self.wrap(t);
        let i = self.span(t);
        let ders = self.basis_ders(i, t, nd);
        let points = self.ctrl.iter().skip(i - self.degree).take(self.degree + 1);
        let mut out = [Vec3::ZERO; 3];
        for (slot, row) in out.iter_mut().zip(&ders).take(nd.min(2) + 1) {
            *slot = points.clone().zip(row).fold(Vec3::ZERO, |s, (&q, &n)| s + q * n);
        }
        out
    }

    pub fn eval(&self, t: f64) -> Vec3 {
        let [point, ..] = self.eval_ders(t, 0);
        point
    }

    pub fn transformed(&self, t: &Transform) -> BSpline {
        BSpline {
            ctrl: self.ctrl.iter().map(|&p| t.point(p)).collect(),
            spans: SpanCache::default(),
            polys: PolyCache::default(),
            ..self.clone()
        }
    }

    fn spans(&self) -> &[Span] {
        self.spans.0.get_or_init(|| {
            let p = self.degree;
            let spans = knot_pairs(&self.knots)
                .skip(p)
                .zip(self.ctrl.windows(p + 1))
                .filter(|&((t0, t1), _)| t1 > t0)
                .map(|((t0, t1), points)| {
                    let centre =
                        points.iter().fold(Vec3::ZERO, |a, &q| a + q) / points.len() as f64;
                    let radius = points.iter().map(|q| q.dist(centre)).fold(0.0, f64::max);
                    Span { t0, t1, centre, radius: radius * (1.0 + BALL_SLACK) + BALL_SLACK }
                })
                .collect();
            Arc::new(spans)
        })
    }

    /// [t0, t1] cut where the polynomial pieces meet, each piece with its knot span, or `None`
    /// where an open spline's range runs past its ends and the end spans extend.
    fn pieces(&self, t0: f64, t1: f64) -> Vec<(f64, f64, Option<&Span>)> {
        let (a, b) = self.domain();
        let mut out = Vec::new();
        let shifts = match self.period() {
            Some(period) => {
                let k0 = ((t0 - b) / period).floor() as i64;
                let k1 = ((t1 - a) / period).ceil() as i64;
                (k0..=k1).map(|k| k as f64 * period).collect()
            }
            None => {
                if t0 < a {
                    out.push((t0, t1.min(a), None));
                }
                if t1 > b {
                    out.push((t0.max(b), t1, None));
                }
                vec![0.0]
            }
        };
        let spans = self.spans();
        for shift in shifts {
            let first = spans.partition_point(|s| s.t1 + shift < t0);
            for s in spans.iter().skip(first) {
                if s.t0 + shift > t1 {
                    break;
                }
                let (lo, hi) = ((s.t0 + shift).max(t0), (s.t1 + shift).min(t1));
                if lo <= hi {
                    out.push((lo, hi, Some(s)));
                }
            }
        }
        out
    }

    /// Parameters strictly inside (t0, t1) where the coordinate along `dir` is stationary: on each
    /// piece that coordinate's derivative is a polynomial of degree below three, solved exactly.
    pub fn extremes(&self, dir: Vec3, t0: f64, t1: f64) -> Vec<f64> {
        self.extremes_along(&[dir], t0, t1, |_| false).into_iter().next().unwrap_or_default()
    }

    /// The tight box of the curve over [t0, t1]: its ends and every coordinate's extremes.
    pub fn bbox(&self, t0: f64, t1: f64) -> Aabb {
        let mut bb = Aabb::EMPTY;
        bb.add(self.eval(t0));
        bb.add(self.eval(t1));
        // A piece whose control points' ball lies in the ends' box cannot widen it.
        let ends = bb;
        let inside = |s: &Span| {
            let (centre, min, max) =
                (s.centre.to_array(), ends.min.to_array(), ends.max.to_array());
            centre
                .into_iter()
                .zip(min)
                .zip(max)
                .all(|((c, lo), hi)| c - s.radius >= lo && c + s.radius <= hi)
        };
        for ts in self.extremes_along(&[Vec3::X, Vec3::Y, Vec3::Z], t0, t1, inside) {
            for t in ts {
                bb.add(self.eval(t));
            }
        }
        bb
    }

    /// [`BSpline::extremes`] along each of `dirs`, sharing the derivative evaluations, on the
    /// pieces whose control points are not all `inside`.
    fn extremes_along(
        &self,
        dirs: &[Vec3],
        t0: f64,
        t1: f64,
        inside: impl Fn(&Span) -> bool,
    ) -> Vec<Vec<f64>> {
        let mut outs = vec![Vec::new(); dirs.len()];
        let der = |t: f64| {
            let [_, tangent, _] = self.eval_ders(t, 1);
            tangent
        };
        let mut last: Option<(f64, Vec3)> = None;
        for (lo, hi, span) in self.pieces(t0, t1) {
            if hi <= lo || span.is_some_and(&inside) {
                continue;
            }
            let d_lo = match last {
                Some((t, d)) if t == lo => d,
                _ => der(lo),
            };
            let (d_mid, d_hi) = (der(0.5 * (lo + hi)), der(hi));
            last = Some((hi, d_hi));
            for (dir, out) in dirs.iter().zip(&mut outs) {
                let (g0, g1, g2) = (d_lo.dot(*dir), d_mid.dot(*dir), d_hi.dot(*dir));
                extremes_on_piece(out, (g0, g1, g2), lo, hi, t0, t1);
            }
        }
        for out in &mut outs {
            out.sort_by(f64::total_cmp);
        }
        outs
    }

    /// The parameter in [t0, t1] of the point nearest `p`, measuring distance across `along`
    /// (a unit vector) when it is given, as an extrusion along it sees the curve. Only the spans
    /// whose ball could hold a nearer point than the best so far are searched.
    pub fn nearest(&self, p: Vec3, along: Option<Vec3>, t0: f64, t1: f64) -> f64 {
        let flat = |v: Vec3| match along {
            Some(d) => v - d * v.dot(d),
            None => v,
        };
        let pieces: Vec<(f64, f64, f64)> = self
            .pieces(t0, t1)
            .into_iter()
            .map(|(lo, hi, span)| {
                let gap = span.map_or(0.0, |s| (flat(s.centre - p).norm() - s.radius).max(0.0));
                (gap, lo, hi)
            })
            .collect();
        let dist = |t: f64| flat(self.eval(t) - p).norm();
        let search = |lo: f64, hi: f64| {
            let mut seed = (f64::INFINITY, lo);
            for i in 0..=NEAREST_SAMPLES {
                let t = lo + (hi - lo) * f64::from(i) / f64::from(NEAREST_SAMPLES);
                let d = dist(t);
                if d < seed.0 {
                    seed = (d, t);
                }
            }
            let mut t = seed.1;
            for _ in 0..NEWTON_STEPS {
                let [c, d1, d2] = self.eval_ders(t, 2);
                let (r, d1, d2) = (flat(c - p), flat(d1), flat(d2));
                let slope = r.dot(d1);
                let curvature = d1.norm2() + r.dot(d2);
                if curvature.abs() < FLAT_SLOPE {
                    break;
                }
                let next = (t - slope / curvature.max(d1.norm2() * 0.1)).clamp(lo, hi);
                let done = (next - t).abs() <= NEWTON_STEP_DONE * (1.0 + t.abs());
                t = next;
                if done {
                    break;
                }
            }
            let d = dist(t);
            if d <= seed.0 { (d, t) } else { seed }
        };
        // The search stops at the first piece whose gap passes the best distance, and the piece of
        // least gap comes first, so no piece whose gap passes that piece's distance is reached.
        let Some(&(_, first_lo, first_hi)) = pieces.iter().min_by(|x, y| x.0.total_cmp(&y.0))
        else {
            return t0;
        };
        let bound = search(first_lo, first_hi).0;
        let mut order: Vec<(f64, f64, f64)> = pieces.into_iter().filter(|q| q.0 <= bound).collect();
        order.sort_by(|x, y| x.0.total_cmp(&y.0));
        let mut best = (f64::INFINITY, t0);
        for (gap, lo, hi) in order {
            if gap > best.0 {
                break;
            }
            let found = search(lo, hi);
            if found.0 < best.0 {
                best = found;
            }
        }
        best.1
    }

    /// The basis function values of every control point at `t`, for collocation: the first
    /// control point they weigh, and their values.
    fn basis_row(&self, t: f64) -> (usize, Vec<f64>) {
        let i = self.span(t);
        let [values, ..] = self.basis_ders(i, t, 0);
        (i - self.degree, values.iter().take(self.degree + 1).copied().collect())
    }

    /// The open spline through `points` in order, at chord-length parameters. Four or more points
    /// give a cubic with not-a-knot ends; three a quadratic; two a line.
    pub fn interpolate(points: &[Vec3]) -> Result<BSpline> {
        let n = points.len();
        if n < 2 {
            return invalid("interpolation needs at least two points");
        }
        let params = chord_params(points, false)?;
        BSpline::interpolate_at(points, &params)
    }

    /// The open spline through `points` at the increasing parameters `params`: a cubic with
    /// not-a-knot ends from four points, a quadratic through three, a line through two.
    pub fn interpolate_at(points: &[Vec3], params: &[f64]) -> Result<BSpline> {
        let n = points.len();
        let (Some(&first), Some(&last)) = (params.first(), params.last()) else {
            return invalid("interpolation needs a parameter for each of at least two points");
        };
        if n < 2 || params.len() != n {
            return invalid("interpolation needs a parameter for each of at least two points");
        }
        let degree = (n - 1).min(3);
        let mut knots = vec![first; degree + 1];
        if degree == 3 {
            // Not-a-knot: no knot at the second point or the second last.
            knots.extend(params.iter().skip(2).take(n - 4));
        }
        knots.extend(repeat_n(last, degree + 1));
        let shape = BSpline::from_parts(degree, knots, vec![Vec3::ZERO; n], false)?;
        let rows = params.iter().map(|&t| shape.basis_row(t)).collect();
        let coordinates: Vec<f64> = points.iter().flat_map(|p| p.to_array()).collect();
        let solved = solve_banded(rows, coordinates, 3)?;
        BSpline::new(degree, shape.knots, points_from(&solved))
    }

    /// The open cubic through `points` at chord-length parameters, with a knot at every inner
    /// point and each end's derivative that of the cubic through its four nearest points (the
    /// parabola through three when there are only three), the ends other modellers give a
    /// spline through points. Two points give a line.
    pub fn interpolate_bessel(points: &[Vec3]) -> Result<BSpline> {
        let n = points.len();
        if n < 3 {
            return BSpline::interpolate(points);
        }
        let params = chord_params(points, false)?;
        let nodes: Vec<(f64, Vec3)> = params.iter().copied().zip(points.iter().copied()).collect();
        let [(t_first, p_first), ref inner @ .., (t_last, p_last)] = *nodes.as_slice() else {
            return invalid("interpolation needs a parameter for each of at least two points");
        };
        let mut knots = vec![t_first; 4];
        knots.extend(inner.iter().map(|&(t, _)| t));
        knots.extend([t_last; 4]);
        let m = n + 2;
        let shape = BSpline::from_parts(3, knots, vec![Vec3::ZERO; m], false)?;
        // Each end's slope from the polynomial through its nearest few points.
        let k = n.min(4);
        let (Some((head, _)), Some((_, tail))) =
            (nodes.split_at_checked(k), nodes.split_at_checked(n - k))
        else {
            return invalid("interpolation needs a parameter for each of at least two points");
        };
        let (start, end) = (lagrange_slope(head, 0), lagrange_slope(tail, k - 1));
        // A collocation row: the basis values (`der` 0) or first derivatives (`der` 1) at `at`.
        let row = |at: f64, der: usize| {
            let i = shape.span(at);
            let [values, slopes, ..] = shape.basis_ders(i, at, der);
            (i - 3, if der == 0 { values } else { slopes }.to_vec())
        };
        let mut rows = Vec::with_capacity(m);
        let mut coordinates: Vec<f64> = Vec::with_capacity(3 * m);
        rows.push(row(t_first, 0));
        coordinates.extend(p_first.to_array());
        rows.push(row(t_first, 1));
        coordinates.extend(start.to_array());
        for &(t, p) in inner {
            rows.push(row(t, 0));
            coordinates.extend(p.to_array());
        }
        rows.push(row(t_last, 1));
        coordinates.extend(end.to_array());
        rows.push(row(t_last, 0));
        coordinates.extend(p_last.to_array());
        let solved = solve_banded(rows, coordinates, 3)?;
        BSpline::new(3, shape.knots, points_from(&solved))
    }

    /// The closed C² cubic through `points` and back to the first, at chord-length parameters.
    pub fn interpolate_periodic(points: &[Vec3]) -> Result<BSpline> {
        if points.len() < 3 {
            return invalid("a periodic spline needs at least three points");
        }
        let params = chord_params(points, true)?;
        BSpline::interpolate_periodic_at(points, &params)
    }

    /// The closed C² cubic through `points` at `params[..n]`, back to the first at `params[n]`,
    /// one period on. Splines through different points at the same parameters share their knots.
    pub fn interpolate_periodic_at(points: &[Vec3], params: &[f64]) -> Result<BSpline> {
        let n = points.len();
        let refuse =
            || invalid("a periodic spline needs three or more points at increasing parameters");
        if n < 3 || params.len() != n + 1 || knot_pairs(params).any(|(a, b)| b <= a) {
            return refuse();
        }
        let (Some((&last, at_points)), Some(&first)) = (params.split_last(), params.first()) else {
            return refuse();
        };
        let period = last - first;
        let degree = 3;
        // Knots u_{-3} .. u_{n+3}, with u_{i+n} = u_i + period: from u_{n-3} of the points' own.
        let knots: Vec<f64> = (-3..=(n as isize + 3))
            .zip(at_points.iter().cycle().skip(n - 3))
            .map(|(i, &u)| u + period * (i.div_euclid(n as isize)) as f64)
            .collect();
        let shape = BSpline::from_parts(degree, knots, vec![Vec3::ZERO; n + degree], false)?;
        // Unknown `q` is control point `q + 1`, which puts each row's largest basis value, at the
        // point's own knot, on the diagonal as the cyclic solve wants.
        let rows = at_points
            .iter()
            .map(|&t| {
                let (first, vals) = shape.basis_row(t);
                ((first + n - 1) % n, vals)
            })
            .collect();
        let coordinates: Vec<f64> = points.iter().flat_map(|p| p.to_array()).collect();
        let solved = points_from(&solve_cyclic(rows, coordinates, 3)?);
        // Control point `i` is unknown `i − 1` (the last for the first), and the first `degree`
        // come again at the end.
        let rotated: Vec<Vec3> =
            solved.last().into_iter().chain(solved.iter().take(n - 1)).copied().collect();
        let ctrl = rotated.iter().chain(rotated.iter().take(degree)).copied().collect();
        BSpline::from_parts(degree, shape.knots, ctrl, true)
    }
}

impl BSpline {
    /// The closed cubic of period `knots[n + 3] − knots[3]` over `n` control points, its knots
    /// `u₋₃ … u_{n+3}` (`u_{i+n} = u_i + period`) and its control points the `n` with the first
    /// three repeated after them, as [`BSpline::interpolate_periodic_at`] makes them.
    pub fn periodic_cubic(knots: Vec<f64>, ctrl: Vec<Vec3>) -> Result<BSpline> {
        let n = ctrl.len().saturating_sub(3);
        if n < 3 || knots.len() != n + 7 || ctrl.get(n..) != ctrl.get(..3) {
            return invalid("a periodic cubic needs n + 7 knots and its first three points again");
        }
        BSpline::from_parts(3, knots, ctrl, true)
    }
}

/// The derivative at node `at` of the polynomial through the `(parameter, point)` `nodes`, in
/// Lagrange form.
fn lagrange_slope(nodes: &[(f64, Vec3)], at: usize) -> Vec3 {
    debug_assert!(at < nodes.len(), "node {at} is one of the nodes");
    let Some(&(t_at, _)) = nodes.get(at) else {
        return Vec3::ZERO;
    };
    let mut d = Vec3::ZERO;
    for (j, &(t_j, p_j)) in nodes.iter().enumerate() {
        let others = nodes.iter().enumerate().filter(|&(m, _)| m != j);
        let l = if j == at {
            others.map(|(_, &(t_m, _))| 1.0 / (t_j - t_m)).sum::<f64>()
        } else {
            others
                .filter(|&(m, _)| m != at)
                .map(|(_, &(t_m, _))| (t_at - t_m) / (t_j - t_m))
                .product::<f64>()
                / (t_j - t_at)
        };
        d += p_j * l;
    }
    d
}

/// The cumulative chord lengths of `points`, from zero; round to the first again when `closed`.
pub fn chord_params(points: &[Vec3], closed: bool) -> Result<Vec<f64>> {
    let count = if closed { points.len() } else { points.len().saturating_sub(1) };
    let mut params = Vec::with_capacity(count + 1);
    let mut length = 0.0;
    params.push(length);
    for (a, b) in points.iter().zip(points.iter().cycle().skip(1)).take(count) {
        let d = a.dist(*b);
        if d <= tol::LINEAR {
            return invalid("interpolation points must be distinct");
        }
        length += d;
        params.push(length);
    }
    Ok(params)
}

/// The stationary points in one piece of a coordinate whose derivative is `g0`, `g1`, `g2` at the
/// piece's start, middle and end: `g(lo + s·(hi − lo)) = g0 + b·s + a·s²` on s ∈ [0, 1].
fn extremes_on_piece(
    out: &mut Vec<f64>,
    (g0, g1, g2): (f64, f64, f64),
    lo: f64,
    hi: f64,
    t0: f64,
    t1: f64,
) {
    let qa = 2.0 * g0 - 4.0 * g1 + 2.0 * g2;
    let qb = -3.0 * g0 + 4.0 * g1 - g2;
    let scale = g0.abs().max(g1.abs()).max(g2.abs());
    if scale == 0.0 {
        return;
    }
    let mut ss = Vec::new();
    if qa.abs() <= NEGLIGIBLE_COEFFICIENT * scale {
        if qb.abs() > NEGLIGIBLE_COEFFICIENT * scale {
            ss.push(-g0 / qb);
        }
    } else {
        let disc = qb * qb - 4.0 * qa * g0;
        if disc >= 0.0 {
            let q = -0.5 * (qb + qb.signum() * disc.sqrt());
            ss.push(q / qa);
            if q != 0.0 {
                ss.push(g0 / q);
            }
        }
    }
    for s in ss {
        let t = lo + s * (hi - lo);
        if (0.0..=1.0).contains(&s) && t > t0 && t < t1 && !out.contains(&t) {
            out.push(t);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ellipse_points(n: usize) -> Vec<Vec3> {
        (0..n)
            .map(|i| {
                let a = std::f64::consts::TAU * i as f64 / n as f64;
                Vec3::new(30.0 * a.cos(), 20.0 * a.sin(), 0.0)
            })
            .collect()
    }

    #[test]
    fn a_box_holds_every_point_of_its_range_and_touches_its_extremes() {
        let pts: Vec<Vec3> = (0..40)
            .map(|i| {
                let t = i as f64 * 0.37;
                Vec3::new(t.cos() * (2.0 + t * 0.1), t.sin() * 1.5, (2.0 * t).sin())
            })
            .collect();
        let c = BSpline::interpolate(&pts).unwrap();
        let (a, b) = c.domain();
        for (t0, t1) in [(a, b), (a + 0.3 * (b - a), a + 0.35 * (b - a)), (0.5, 0.5 + 1e-3)] {
            let bb = c.bbox(t0, t1);
            let mut dense = Aabb::EMPTY;
            for i in 0..=4000 {
                dense.add(c.eval(t0 + (t1 - t0) * i as f64 / 4000.0));
            }
            for k in 0..3 {
                assert!(bb.min[k] <= dense.min[k] + 1e-12 && bb.max[k] >= dense.max[k] - 1e-12);
                assert!(dense.min[k] - bb.min[k] < 1e-4 && bb.max[k] - dense.max[k] < 1e-4);
            }
        }
    }

    #[test]
    fn open_cubic_passes_through_its_points_and_its_derivative_matches_differences() {
        let pts: Vec<Vec3> =
            (0..7).map(|i| Vec3::new(i as f64, (i as f64 * 0.7).sin() * 3.0, 0.0)).collect();
        let s = BSpline::interpolate(&pts).unwrap();
        let params = chord_params(&pts, false).unwrap();
        for (p, &t) in pts.iter().zip(&params) {
            assert!(s.eval(t).dist(*p) < 1e-12);
        }
        let t = 2.3;
        let h = 1e-6;
        let [_, d1, d2] = s.eval_ders(t, 2);
        let fd = (s.eval(t + h) - s.eval(t - h)) / (2.0 * h);
        let fd2 = (s.eval(t + h) - s.eval(t) * 2.0 + s.eval(t - h)) / (h * h);
        assert!(d1.dist(fd) < 1e-7, "{d1:?} {fd:?}");
        assert!(d2.dist(fd2) < 1e-3, "{d2:?} {fd2:?}");
    }

    #[test]
    fn periodic_cubic_closes_smoothly_through_ellipse_points() {
        let pts = ellipse_points(72);
        let s = BSpline::interpolate_periodic(&pts).unwrap();
        let (a, b) = s.domain();
        let params = chord_params(&pts, true).unwrap();
        for (p, &t) in pts.iter().zip(&params) {
            assert!(s.eval(t).dist(*p) < 1e-12);
        }
        let [p0, d0, dd0] = s.eval_ders(a, 2);
        let [p1, d1, dd1] = s.eval_ders(b - 1e-12, 2);
        assert!(p0.dist(p1) < 1e-9);
        assert!(d0.dist(d1) < 1e-9);
        assert!(dd0.dist(dd1) < 1e-6);
        // Between samples the interpolant stays within 1e-5 of the ellipse.
        for i in 0..720 {
            let q = s.eval(a + (b - a) * (i as f64 + 0.5) / 720.0);
            let r = (q.x / 30.0).powi(2) + (q.y / 20.0).powi(2);
            assert!((r - 1.0).abs() < 2e-5, "{r}");
        }
    }

    #[test]
    fn ends_take_the_tangent_of_the_cubic_through_their_four_nearest_points() {
        let pts: Vec<Vec3> = (0..6)
            .map(|i| Vec3::new(i as f64, (i as f64 * 0.9).sin() * 2.0, 0.1 * i as f64))
            .collect();
        let s = BSpline::interpolate_bessel(&pts).unwrap();
        assert_eq!(s.control_points().len(), pts.len() + 2);
        let t = chord_params(&pts, false).unwrap();
        for (p, &u) in pts.iter().zip(&t) {
            assert!(s.eval(u).dist(*p) < 1e-12);
        }
        // Four points determine the cubic, so their own spline is it and its end tangents match.
        let four = BSpline::interpolate_bessel(&pts[..4]).unwrap();
        let tf = chord_params(&pts[..4], false).unwrap();
        assert!(s.eval_ders(t[0], 1)[1].dist(four.eval_ders(tf[0], 1)[1]) < 1e-12);
        let mid = 0.5 * (tf[1] + tf[2]);
        let lagrange: Vec3 = (0..4)
            .map(|j| {
                let l: f64 =
                    (0..4).filter(|&m| m != j).map(|m| (mid - tf[m]) / (tf[j] - tf[m])).product();
                pts[j] * l
            })
            .fold(Vec3::ZERO, |a, b| a + b);
        assert!(four.eval(mid).dist(lagrange) < 1e-12);
        let tail = BSpline::interpolate_bessel(&pts[2..]).unwrap();
        let tt = chord_params(&pts[2..], false).unwrap();
        let end = s.eval_ders(t[5], 1)[1];
        assert!(end.dist(tail.eval_ders(tt[3], 1)[1]) < 1e-12);
        let three = BSpline::interpolate_bessel(&pts[..3]).unwrap();
        assert!(three.eval(t[1]).dist(pts[1]) < 1e-12);
    }

    fn wobbly_ring() -> BSpline {
        let pts: Vec<Vec3> = (0..90)
            .map(|i| {
                let a = std::f64::consts::TAU * i as f64 / 90.0;
                let r = 30.0 + 1.5 * (9.0 * a).cos();
                Vec3::new(r * a.cos(), r * a.sin(), 0.0)
            })
            .collect();
        BSpline::interpolate_periodic(&pts).unwrap()
    }

    #[test]
    fn span_polynomials_agree_with_the_basis_recurrence() {
        // The point and both derivatives, at every knot, inside each span and past a periodic
        // spline's ends: an interpolated ring, an open cubic, a quadratic with a double knot (C⁰
        // there) and a polyline.
        let pts = |n: usize| -> Vec<Vec3> {
            (0..n)
                .map(|i| {
                    let x = i as f64;
                    Vec3::new(x * 3.1, (x * 0.7).sin() * 4.0, x * x * 0.05)
                })
                .collect()
        };
        let mut splines = vec![wobbly_ring(), BSpline::interpolate(&pts(11)).unwrap()];
        let ctrl = pts(6);
        splines.push(
            BSpline::new(2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 2.5, 4.0, 4.0, 4.0], ctrl.clone())
                .unwrap(),
        );
        splines.push(BSpline::new(1, vec![0.0, 0.0, 1.0, 2.0, 3.0, 4.5, 5.0, 5.0], ctrl).unwrap());
        for c in &splines {
            let (a, b) = c.domain();
            let mut ts: Vec<f64> = c.knots().to_vec();
            ts.extend((0..=97).map(|k| a + (b - a) * (k as f64 / 97.0)));
            if c.is_periodic() {
                ts.extend([a - 0.3 * (b - a), b + 0.7 * (b - a)]);
            }
            for &t in &ts {
                if !c.is_periodic() && (t < a || t > b) {
                    continue;
                }
                let got = c.eval_ders(t, 2);
                let want = c.eval_ders_by_basis(t, 2);
                for (k, tol) in [1e-12, 1e-11, 1e-10].into_iter().enumerate() {
                    let size = 1.0 + want[k].norm();
                    assert!(
                        got[k].dist(want[k]) <= tol * size,
                        "order {k} at {t}: {:?} against {:?}",
                        got[k],
                        want[k]
                    );
                }
            }
        }
    }

    #[test]
    fn nearest_points_match_dense_sampling() {
        let s = wobbly_ring();
        let (a, b) = s.domain();
        let ranges = [(a, b), (a - 7.0, a + 5.0), (a + 40.0, a + 41.0)];
        let points = [
            Vec3::new(0.3, -0.2, 0.0),
            Vec3::new(31.0, 2.0, 4.0),
            Vec3::new(-50.0, 10.0, -3.0),
            Vec3::new(5.0, 28.0, 1.0),
        ];
        for (t0, t1) in ranges {
            for p in points {
                for along in [None, Some(Vec3::Z)] {
                    let flat = |v: Vec3| along.map_or(v, |d: Vec3| v - d * v.dot(d));
                    let dist = |t: f64| flat(s.eval(t) - p).norm();
                    let t = s.nearest(p, along, t0, t1);
                    assert!(t >= t0 && t <= t1);
                    let dense = (0..=4000)
                        .map(|i| dist(t0 + (t1 - t0) * i as f64 / 4000.0))
                        .fold(f64::INFINITY, f64::min);
                    assert!(dist(t) <= dense + 1e-12, "{p:?} {t0} {t1}");
                }
            }
        }
    }

    #[test]
    fn extremes_are_every_stationary_point() {
        let s = wobbly_ring();
        let (a, b) = s.domain();
        for dir in [Vec3::X, Vec3::new(0.6, 0.8, 0.0)] {
            for (t0, t1) in [(a, b), (a - 7.0, a + 5.0)] {
                let ts = s.extremes(dir, t0, t1);
                let g = |t: f64| s.eval_ders(t, 1)[1].dot(dir);
                for &t in &ts {
                    assert!(t > t0 && t < t1);
                    assert!(g(t).abs() < 1e-9, "{t} {}", g(t));
                }
                // A derivative of exactly zero at an end of the window is no change inside it.
                let n = 5000;
                let changes = (0..n)
                    .filter(|&i| {
                        let x = |i: usize| t0 + (t1 - t0) * i as f64 / n as f64;
                        let (g0, g1) = (g(x(i)), g(x(i + 1)));
                        let at_an_end = (i == 0 && g0 == 0.0) || (i + 1 == n && g1 == 0.0);
                        !at_an_end && (g0 < 0.0) != (g1 < 0.0)
                    })
                    .count();
                assert_eq!(ts.len(), changes);
            }
        }
    }

    #[test]
    fn three_points_give_a_quadratic() {
        let pts = [Vec3::ZERO, Vec3::new(1.0, 1.0, 0.0), Vec3::new(2.0, 0.0, 0.0)];
        let s = BSpline::interpolate(&pts).unwrap();
        assert_eq!(s.degree(), 2);
        assert!(s.eval(s.domain().1).dist(pts[2]) < 1e-14);
    }
}
