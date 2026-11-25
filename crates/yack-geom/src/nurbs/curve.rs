// yack: rational B-spline curves of any degree, as STEP files carry them.
// Adapted from springrep@a03e8a1 src/geom/nurbs.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/geom/nurbs.rs to yack-geom (M3 W3); split out of
// nurbs.rs into its own module (M3 hardening). M3 hardening: a span's control points and weights
// come by iterators instead of index arithmetic; the same arithmetic in the same order.
//! Rational B-spline curves of any degree, as STEP files carry them.

use super::basis::{Basis, check_weights};
use crate::{
    Result,
    math::{Aabb, Point3, Transform, Vec3},
    tol,
};

/// A rational (or, without weights, polynomial) B-spline curve.
#[derive(Clone, Debug, PartialEq)]
pub struct NurbsCurve {
    basis: Basis,
    ctrl: Vec<Point3>,
    weights: Option<Vec<f64>>,
    period: Option<f64>,
}

/// `weights`, or `None` when they are all equal: the curve or surface is then polynomial.
fn unequal(weights: Option<Vec<f64>>) -> Option<Vec<f64>> {
    weights.filter(|w| w.first().is_some_and(|&first| w.iter().any(|&x| x != first)))
}

impl NurbsCurve {
    /// The curve from its parts; weights all 1 when `weights` is `None`. The curve is periodic when
    /// its two ends meet within the linear tolerance.
    pub fn new(
        degree: usize,
        knots: Vec<f64>,
        ctrl: Vec<Point3>,
        weights: Option<Vec<f64>>,
    ) -> Result<NurbsCurve> {
        let basis = Basis::new(degree, knots, ctrl.len())?;
        check_weights(&weights, ctrl.len())?;
        let weights = unequal(weights);
        let mut c = NurbsCurve { basis, ctrl, weights, period: None };
        let (a, b) = c.domain();
        if c.eval(a).dist(c.eval(b)) <= tol::LINEAR {
            c.period = Some(b - a);
        }
        Ok(c)
    }

    /// The curve exactly as [`NurbsCurve::new`] made it, from its parts and its period.
    pub fn restore(
        degree: usize,
        knots: Vec<f64>,
        ctrl: Vec<Point3>,
        weights: Option<Vec<f64>>,
        period: Option<f64>,
    ) -> Result<NurbsCurve> {
        let basis = Basis::new(degree, knots, ctrl.len())?;
        check_weights(&weights, ctrl.len())?;
        Ok(NurbsCurve { basis, ctrl, weights, period })
    }

    pub fn degree(&self) -> usize {
        self.basis.degree
    }

    pub fn knots(&self) -> &[f64] {
        &self.basis.knots
    }

    pub fn control_points(&self) -> &[Point3] {
        &self.ctrl
    }

    /// The weights, or `None` when every weight is equal (the curve is polynomial).
    pub fn weights(&self) -> Option<&[f64]> {
        self.weights.as_deref()
    }

    pub fn domain(&self) -> (f64, f64) {
        self.basis.domain()
    }

    pub fn period(&self) -> Option<f64> {
        self.period
    }

    pub fn breakpoints(&self, t0: f64, t1: f64) -> Vec<f64> {
        self.basis.breakpoints(t0, t1, self.period)
    }

    /// The control points that shape the curve on span `i`, with their weights (1 on a
    /// polynomial curve).
    fn span_points(&self, i: usize) -> impl Iterator<Item = (Point3, f64)> + '_ {
        // The weights number as the control points (`check_weights`), so every one is there.
        let weight =
            |k: usize| self.weights.as_ref().map_or(1.0, |w| w.get(k).copied().unwrap_or(f64::NAN));
        debug_assert!(
            self.weights.as_ref().is_none_or(|w| w.len() == self.ctrl.len()),
            "a weight for every control point"
        );
        let p = self.basis.degree;
        self.ctrl.iter().enumerate().skip(i - p).take(p + 1).map(move |(k, &q)| (q, weight(k)))
    }

    /// The box of the control points that shape the curve at `t`. Its weights being positive, the
    /// curve over the knot span there lies in their convex hull, and so in the box.
    pub(crate) fn span_box(&self, t: f64) -> Aabb {
        let t = self.basis.wrap(t, self.period);
        let mut bb = Aabb::EMPTY;
        for (q, _) in self.span_points(self.basis.span(t)) {
            bb.add(q);
        }
        bb
    }

    /// The point and its first two derivatives at `t`.
    pub fn d2(&self, t: f64) -> (Point3, Vec3, Vec3) {
        let t = self.basis.wrap(t, self.period);
        let i = self.basis.span(t);
        self.basis.ders_then(i, t, 2, |n| self.d2_from(n, i))
    }

    /// [`NurbsCurve::d2`] from the basis values and derivatives `n` on span `i`.
    fn d2_from(&self, [n0, n1, n2]: [&[f64]; 3], i: usize) -> (Point3, Vec3, Vec3) {
        let mut a = [Vec3::ZERO; 3];
        let mut w = [0.0; 3];
        let ders = n0.iter().zip(n1).zip(n2);
        for ((q, wt), ((&d0, &d1), &d2)) in self.span_points(i).zip(ders) {
            let pw = q * wt;
            for ((a, w), n) in a.iter_mut().zip(&mut w).zip([d0, d1, d2]) {
                *a += pw * n;
                *w += wt * n;
            }
        }
        let ([a0, a1, a2], [w0, w1, w2]) = (a, w);
        let c = a0 / w0;
        let d1 = (a1 - c * w1) / w0;
        let d2 = (a2 - d1 * (2.0 * w1) - c * w2) / w0;
        (c, d1, d2)
    }

    pub fn eval(&self, t: f64) -> Point3 {
        self.d2(t).0
    }

    /// The curve's denominator at `t`, the blend of its weights there: 1 on a polynomial curve.
    /// The curve times it is a polynomial over each piece, where the curve itself is not.
    pub fn weight(&self, t: f64) -> f64 {
        if self.weights.is_none() {
            return 1.0;
        }
        let t = self.basis.wrap(t, self.period);
        let i = self.basis.span(t);
        self.basis.ders_then(i, t, 0, |[n0, ..]| {
            self.span_points(i).zip(n0).map(|((_, w), n)| w * n).sum()
        })
    }

    pub fn transformed(&self, t: &Transform) -> NurbsCurve {
        NurbsCurve { ctrl: self.ctrl.iter().map(|&p| t.point(p)).collect(), ..self.clone() }
    }
}
