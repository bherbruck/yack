// yack: a curve's polynomial pieces, and the roots of a function of its point over them.
// Adapted from springrep@a03e8a1 src/boolean/csi.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: split out of csi.rs (M3 hardening): a piece's parameter and scale as data (an enum each)
// instead of boxed closures; the same arithmetic in the same order.
//! A curve's polynomial pieces, and the roots of a function of its point over them. A function of
//! a spline's point is a polynomial over each knot span once multiplied by the weight enough times,
//! so its stationary points split the span into stretches holding one root at most: no root is
//! stepped over however close two lie.

use std::f64::consts::TAU;

use super::roots::{fitted, roots_between, spans, stationary_points, with_ends};
use crate::{
    geom::{Curve, NurbsCurve, Ruled},
    tol,
};

/// A stretch of a curve's parameter over which the curve's point, times a positive scale, is a
/// polynomial: the parameter at `s` in [0, 1], the scale there, and the polynomial's degree in `s`.
/// A function of order `k` in the point (1 for a distance from a plane, 2 for a squared radius) is
/// then a polynomial of degree `k·degree` once multiplied by the scale `k` times over.
pub(super) struct Piece<'a> {
    param: Param,
    scale: Scale<'a>,
    pub(super) degree: usize,
}

/// How a piece's parameter follows `s` in [0, 1].
#[derive(Clone, Copy)]
enum Param {
    /// Evenly from the first to the second.
    Span(f64, f64),
    /// A half turn of a conic about the angle `middle`, as `middle + 2·atan(2s − 1)`: the conic's
    /// point is then a quadratic over `1 + (2s − 1)²`.
    HalfTurn { middle: f64 },
}

/// A piece's scale, which makes its point a polynomial.
#[derive(Clone, Copy)]
enum Scale<'a> {
    One,
    /// A rational spline's weight at the piece's parameter.
    Weight(&'a NurbsCurve),
    /// A conic's half turn's denominator, `1 + (2s − 1)²`.
    HalfTurn,
    /// The product of a ruled surface's two curves' weights, at the piece's parameter as the
    /// surface's own.
    Ruled(&'a Ruled),
}

/// The parameter a share `u` along `(lo, hi)`.
pub(super) fn along((lo, hi): (f64, f64), u: f64) -> f64 {
    lo + u * (hi - lo)
}

/// A curve's weight at `t`: one but on a rational spline.
fn weight(c: &Curve, t: f64) -> f64 {
    match c {
        Curve::Nurbs(nc) => nc.weight(t),
        _ => 1.0,
    }
}

impl Piece<'_> {
    /// The curve's parameter at `s`.
    pub(super) fn param(&self, s: f64) -> f64 {
        match self.param {
            Param::Span(a, b) => a + (b - a) * s,
            Param::HalfTurn { middle } => middle + 2.0 * (2.0 * s - 1.0).atan(),
        }
    }

    /// The scale at `s`.
    fn scale(&self, s: f64) -> f64 {
        match self.scale {
            Scale::One => 1.0,
            Scale::Weight(nc) => nc.weight(self.param(s)),
            Scale::HalfTurn => 1.0 + (2.0 * s - 1.0).powi(2),
            Scale::Ruled(r) => {
                let u = self.param(s);
                weight(&r.a, along(r.a_range, u)) * weight(&r.b, along(r.b_range, u))
            }
        }
    }
}

/// The pieces between consecutive `cuts`, of degree `degree`, scaled by `scale`.
fn span_pieces<'a>(cuts: &[f64], degree: usize, scale: Scale<'a>) -> Vec<Piece<'a>> {
    spans(cuts).map(|(a, b)| Piece { param: Param::Span(a, b), scale, degree }).collect()
}

/// The pieces of `c` over `(lo, hi)`: a spline's own, each a polynomial, or a rational one's with
/// its denominator for scale; a line in one; a circle or an ellipse in two half turns, each a
/// quadratic over `1 + τ²` in the tangent `τ` of the half angle, whatever the range. `None` for a
/// line without ends and for an intersection curve, which is no polynomial in any parameter.
pub(super) fn pieces(c: &Curve, (lo, hi): (f64, f64)) -> Option<Vec<Piece<'_>>> {
    let knot_spans = |degree: usize, scale| {
        span_pieces(&with_ends(c.breakpoints(lo, hi), (lo, hi)), degree, scale)
    };
    match c {
        Curve::Line(_) | Curve::BSpline(_) | Curve::Nurbs(_)
            if !(lo.is_finite() && hi.is_finite()) =>
        {
            None
        }
        Curve::Line(_) => Some(knot_spans(1, Scale::One)),
        Curve::BSpline(b) => Some(knot_spans(b.degree(), Scale::One)),
        Curve::Nurbs(nc) => Some(knot_spans(nc.degree(), Scale::Weight(nc))),
        Curve::Circle(_) | Curve::Ellipse(_) => Some(
            [TAU / 4.0, 3.0 * TAU / 4.0]
                .into_iter()
                .map(|middle| Piece {
                    param: Param::HalfTurn { middle },
                    scale: Scale::HalfTurn,
                    degree: 2,
                })
                .collect(),
        ),
        Curve::Intersection(_) => None,
    }
}

/// The pieces of a ruled surface's parameter between consecutive `cuts`, of degree `degree`, each
/// scaled by both curves' weights.
pub(super) fn ruled_pieces<'a>(r: &'a Ruled, cuts: &[f64], degree: usize) -> Vec<Piece<'a>> {
    span_pieces(cuts, degree, Scale::Ruled(r))
}

/// Where a function of a curve's point vanishes, over the curve's `pieces`. `raw` is the function
/// in any unit, a polynomial over each piece once scaled, of order `order` in the point; `mm` has
/// its sign and is a distance in mm. The polynomial's stationary points split each piece into
/// stretches with one crossing at most and mark every place the function can touch zero, so no
/// crossing is stepped over however close two lie. Also whether the function vanishes all along.
pub(super) fn piece_roots(
    pieces: &[Piece],
    order: usize,
    raw: &impl Fn(f64) -> f64,
    mm: &mut impl FnMut(f64) -> f64,
) -> (Vec<f64>, bool) {
    let (mut roots, mut flat) = (Vec::new(), true);
    for piece in pieces {
        let n = piece.degree * order;
        let samples: Vec<f64> = (0..=n).map(|j| j as f64 / n.max(1) as f64).collect();
        let values: Vec<f64> = samples
            .iter()
            .map(|&s| raw(piece.param(s)) * piece.scale(s).powi(order as i32))
            .collect();
        flat &= samples.iter().all(|&s| mm(piece.param(s)).abs() <= tol::LINEAR);
        let mut knots = vec![piece.param(0.0)];
        if let Some(coefficients) = fitted(&values) {
            knots.extend(
                stationary_points(&coefficients, 0.0, 1.0).into_iter().map(|s| piece.param(s)),
            );
        }
        knots.push(piece.param(1.0));
        roots.extend(roots_between(mm, &knots));
    }
    (roots, flat)
}
