// yack: how finely a curve is sampled for a mesh: the chord limits, and an edge's sample parameters.
// Adapted from springrep@a03e8a1 src/mesh/mod.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from mesh/mod.rs to geom/chord.rs (M2), so rays and distances, below the mesher, can
// sample curves alike; mesh re-exports them. Paths only.
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
        MeshOptions {
            linear,
            angular: MeshOptions::ANGULAR,
        }
    }
}

/// The parameters at which an edge's curve is sampled.
pub fn edge_params(curve: &Curve, t0: f64, t1: f64, opt: &MeshOptions) -> Vec<f64> {
    match curve {
        Curve::Line(_) => vec![t0, t1],
        Curve::Circle(c) => {
            let chord = if opt.linear < c.radius {
                2.0 * (1.0 - opt.linear / c.radius).acos()
            } else {
                f64::INFINITY
            };
            // A chord no longer than the deflection cannot stray by more, so the turn alone never
            // splits one: a hair-wide circle at a fine angle would put its samples within the
            // tolerance of each other.
            let step = chord
                .min(opt.angular.max((opt.linear / c.radius).min(FRAC_PI_2)))
                .max(1e-4);
            let n = ((t1 - t0) / step).ceil().max(1.0) as usize;
            (0..=n)
                .map(|i| t0 + (t1 - t0) * i as f64 / n as f64)
                .collect()
        }
        _ => {
            let mut cuts = vec![t0];
            // A knot that rounding puts beside an end would crowd samples onto the end's vertex.
            let margin = 1e-9 * (t1 - t0);
            cuts.extend(
                curve
                    .breakpoints(t0, t1)
                    .into_iter()
                    .filter(|&k| k > t0 + margin && k < t1 - margin),
            );
            cuts.push(t1);
            let mut out = vec![t0];
            for w in cuts.windows(2) {
                for k in 0..4 {
                    let a = w[0] + (w[1] - w[0]) * k as f64 / 4.0;
                    let b = w[0] + (w[1] - w[0]) * (k + 1) as f64 / 4.0;
                    subdivide(curve, a, b, opt, 0, &mut out);
                }
            }
            coarsen(curve, &out, opt)
        }
    }
}

/// The fewest of the `dense` parameters whose chords stay within the deflection limits, tested
/// at every dense parameter and the middles between them. A spline has a break at every knot, and
/// an intersection curve has many knots, so sampling each knot span alone is far too fine.
pub fn coarsen(curve: &Curve, dense: &[f64], opt: &MeshOptions) -> Vec<f64> {
    let n = dense.len();
    if n <= 2 {
        return dense.to_vec();
    }
    let pts: Vec<Point3> = dense.iter().map(|&t| curve.eval(t)).collect();
    let mids: Vec<Point3> = dense
        .windows(2)
        .map(|w| curve.eval(0.5 * (w[0] + w[1])))
        .collect();
    let tangents: Vec<Vec3> = dense.iter().map(|&t| curve.d1(t).1).collect();
    let fits = |i: usize, j: usize| {
        if tangents[i].angle(tangents[j]) > opt.angular {
            return false;
        }
        let (a, b) = (pts[i], pts[j]);
        (i..j).all(|k| {
            (k == i || segment_dist(pts[k], a, b) <= opt.linear)
                && segment_dist(mids[k], a, b) <= opt.linear
        })
    };
    let mut out = vec![dense[0]];
    let mut i = 0;
    while i < n - 1 {
        let mut j = i + 1;
        while j + 1 < n && fits(i, j + 1) {
            j += 1;
        }
        out.push(dense[j]);
        i = j;
    }
    out
}

fn segment_dist(p: Point3, a: Point3, b: Point3) -> f64 {
    let ab = b - a;
    let len2 = ab.dot(ab);
    let s = if len2 > 0.0 {
        ((p - a).dot(ab) / len2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    p.dist(a + ab * s)
}

fn subdivide(c: &Curve, a: f64, b: f64, opt: &MeshOptions, depth: u32, out: &mut Vec<f64>) {
    let (pa, da) = c.d1(a);
    let (pb, db) = c.d1(b);
    let chord = pb - pa;
    let dev = [0.25, 0.5, 0.75]
        .iter()
        .map(|f| {
            let q = c.eval(a + (b - a) * f);
            let along = pa + chord * *f;
            q.dist(along)
        })
        .fold(0.0, f64::max);
    let turn = da.angle(db);
    let long = chord.norm() > opt.linear;
    if depth < 24 && (dev > opt.linear || (long && turn > opt.angular)) {
        let m = 0.5 * (a + b);
        subdivide(c, a, m, opt, depth + 1, out);
        subdivide(c, m, b, opt, depth + 1, out);
    } else {
        out.push(b);
    }
}
