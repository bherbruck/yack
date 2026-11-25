// yack: how finely a curve is sampled for a mesh: the chord limits, and an edge's sample parameters.
// Adapted from springrep@a03e8a1 src/mesh/mod.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from mesh/mod.rs to geom/chord.rs (M2), so rays and distances, below the mesher, can
// sample curves alike; mesh re-exports them. M3 hardening: named steps and samples, iterators and
// slice patterns instead of indexing; the same arithmetic in the same order.
//! How finely a curve is sampled for a mesh: [`MeshOptions`], the chord limits, and the parameters
//! [`edge_params`] samples an edge's curve at.

use std::f64::consts::FRAC_PI_2;

use super::Curve;
use crate::math::{Point3, Vec3};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MeshOptions {
    /// The largest distance from a triangle or segment to the exact geometry, in mm.
    pub linear: f64,
    /// The largest turn of the normal or tangent across one triangle or segment, in radians.
    pub angular: f64,
}

impl MeshOptions {
    /// The turn allowed across one triangle where only a distance is asked for, in radians.
    pub const ANGULAR: f64 = 0.5;

    /// A mesh within `linear` of the exact geometry, at the kernel's own limit on the turn.
    pub fn within(linear: f64) -> MeshOptions {
        MeshOptions { linear, angular: MeshOptions::ANGULAR }
    }
}

/// The smallest angular step along a circle, in radians: it caps a hair-wide circle's samples.
const MIN_CIRCLE_STEP: f64 = 1e-4; // tol: a sample-count cap, not a geometric tolerance

/// A knot closer than this fraction of the range to an end is the end, moved by rounding.
const END_KNOT_MARGIN: f64 = 1e-9; // tol: relative parameter rounding at the range ends

/// Each knot span is first cut into this many equal pieces before adaptive subdivision.
const SPAN_PIECES: u32 = 4;

/// The deepest an adaptive subdivision goes: 2^-24 of a starting piece.
const MAX_DEPTH: u32 = 24;

/// The parameters at which an edge's curve is sampled.
pub fn edge_params(curve: &Curve, t0: f64, t1: f64, options: &MeshOptions) -> Vec<f64> {
    match curve {
        Curve::Line(_) => vec![t0, t1],
        Curve::Circle(circle) => circle_params(circle.radius, t0, t1, options),
        _ => {
            let margin = END_KNOT_MARGIN * (t1 - t0);
            let inner_knots = curve
                .breakpoints(t0, t1)
                .into_iter()
                .filter(|&k| k > t0 + margin && k < t1 - margin);
            let cuts: Vec<f64> =
                std::iter::once(t0).chain(inner_knots).chain(std::iter::once(t1)).collect();
            let mut dense = vec![t0];
            for (start, end) in consecutive(&cuts) {
                let length = end - start;
                let at = |piece: u32| start + length * f64::from(piece) / f64::from(SPAN_PIECES);
                for piece in 0..SPAN_PIECES {
                    subdivide(curve, at(piece), at(piece + 1), options, 0, &mut dense);
                }
            }
            coarsen(curve, &dense, options)
        }
    }
}

/// Equal steps along a circle of `radius`, each within both chord limits.
fn circle_params(radius: f64, t0: f64, t1: f64, options: &MeshOptions) -> Vec<f64> {
    let deflection_step = if options.linear < radius {
        2.0 * (1.0 - options.linear / radius).acos()
    } else {
        f64::INFINITY
    };
    // A chord no longer than the deflection cannot stray by more, so the turn alone never splits
    // one: a hair-wide circle at a fine angle would put its samples within the tolerance of each
    // other.
    let turn_step = options.angular.max((options.linear / radius).min(FRAC_PI_2));
    let step = deflection_step.min(turn_step).max(MIN_CIRCLE_STEP);
    let steps = ((t1 - t0) / step).ceil().max(1.0) as usize;
    (0..=steps).map(|i| t0 + (t1 - t0) * i as f64 / steps as f64).collect()
}

/// Each value paired with the next.
fn consecutive(values: &[f64]) -> impl Iterator<Item = (f64, f64)> + '_ {
    values.iter().copied().zip(values.iter().copied().skip(1))
}

/// A dense sample of the curve: its point and tangent.
struct Sample {
    point: Point3,
    tangent: Vec3,
}

/// The fewest of the `dense` parameters whose chords stay within the deflection limits, tested
/// at every dense parameter and the middles between them. A spline has a break at every knot, and
/// an intersection curve has many knots, so sampling each knot span alone is far too fine.
fn coarsen(curve: &Curve, dense: &[f64], options: &MeshOptions) -> Vec<f64> {
    let [first, _, _, ..] = dense else {
        return dense.to_vec();
    };
    let samples: Vec<Sample> =
        dense.iter().map(|&t| Sample { point: curve.eval(t), tangent: curve.d1(t).1 }).collect();
    let middles: Vec<Point3> = consecutive(dense).map(|(a, b)| curve.eval(0.5 * (a + b))).collect();
    // Whether one chord from sample `from` to sample `to` stands for every sample and middle
    // between them.
    let chord_fits = |from: usize, to: usize| {
        let Some([start, inner @ .., end]) = samples.get(from..=to) else {
            return false;
        };
        if start.tangent.angle(end.tangent) > options.angular {
            return false;
        }
        let near = |p: Point3| segment_distance(p, start.point, end.point) <= options.linear;
        inner.iter().all(|s| near(s.point))
            && middles.get(from..to).is_some_and(|span| span.iter().all(|&m| near(m)))
    };
    let last = dense.len() - 1;
    let mut kept = vec![*first];
    let mut from = 0;
    while from < last {
        let to =
            (from + 2..=last).take_while(|&to| chord_fits(from, to)).last().unwrap_or(from + 1);
        kept.extend(dense.get(to));
        from = to;
    }
    kept
}

/// The distance from `p` to the segment from `a` to `b`.
fn segment_distance(p: Point3, a: Point3, b: Point3) -> f64 {
    let ab = b - a;
    let length_squared = ab.dot(ab);
    let s =
        if length_squared > 0.0 { ((p - a).dot(ab) / length_squared).clamp(0.0, 1.0) } else { 0.0 };
    p.dist(a + ab * s)
}

/// Halves `a..b` until each piece's chord is within the limits (or [`MAX_DEPTH`] is reached), and
/// pushes the end of each piece in order.
fn subdivide(curve: &Curve, a: f64, b: f64, options: &MeshOptions, depth: u32, out: &mut Vec<f64>) {
    let (start, start_tangent) = curve.d1(a);
    let (end, end_tangent) = curve.d1(b);
    let chord = end - start;
    let deviation = [0.25, 0.5, 0.75]
        .iter()
        .map(|&f| curve.eval(a + (b - a) * f).dist(start + chord * f))
        .fold(0.0, f64::max);
    let turn = start_tangent.angle(end_tangent);
    let long = chord.norm() > options.linear;
    if depth < MAX_DEPTH && (deviation > options.linear || (long && turn > options.angular)) {
        let middle = 0.5 * (a + b);
        subdivide(curve, a, middle, options, depth + 1, out);
        subdivide(curve, middle, b, options, depth + 1, out);
    } else {
        out.push(b);
    }
}
