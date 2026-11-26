// yack: whether a fitted intersection curve turns back on itself.
// Adapted from springrep@a03e8a1 src/check.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/check.rs to yack-check (M3 W6); split out of lib.rs into its
// own module (M3 hardening); iterators instead of indexing, named constants.
//! Whether a fitted intersection curve turns back on itself.

use crate::geom::Curve;

/// Where an intersection curve turns back on itself over `range`: its direction at two
/// neighbouring samples, a quarter of a knot span apart, more than a right angle apart. A fitted
/// branch turns little across one span, so that is a fold, and a face bounded by it touches itself.
/// Only fitted curves are tested; an imported spline may have a cusp its designer drew.
/// A curve moving slower than this has no direction to fold back on.
const STILL: f64 = 1e-12; // tol: a speed at rounding, not a length

pub(crate) fn fold(c: &Curve, range: (f64, f64)) -> Option<crate::math::Point3> {
    let Curve::Intersection(ic) = c else {
        return None;
    };
    let sp = &ic.approx;
    let (t0, t1) = range;
    let mut cuts: Vec<f64> = vec![t0];
    cuts.extend(sp.breakpoints(t0, t1));
    cuts.push(t1);
    cuts.dedup();
    let mut prev: Option<crate::math::Vec3> = None;
    for (lo, hi) in cuts.iter().copied().zip(cuts.iter().copied().skip(1)) {
        for j in 0..4 {
            let t = lo + (hi - lo) * j as f64 / 4.0;
            let [p, d] = {
                let [p, d, _] = sp.eval_ders(t, 1);
                [p, d]
            };
            if d.norm() <= STILL {
                continue;
            }
            if prev.is_some_and(|q| q.dot(d) < 0.0) {
                return Some(p);
            }
            prev = Some(d);
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::{BSpline, IntersectionCurve, Plane, Surface};
    use crate::math::{Frame, Vec3};

    fn fitted(points: &[Vec3]) -> Curve {
        let plane =
            |z: Vec3| Surface::Plane(Plane { frame: Frame::from_axis(Vec3::ZERO, z).unwrap() });
        Curve::Intersection(Box::new(IntersectionCurve {
            approx: BSpline::interpolate(points).unwrap(),
            surfaces: [plane(Vec3::Z), plane(Vec3::Y)],
        }))
    }

    #[test]
    fn a_fitted_curve_that_doubles_back_folds_and_one_that_bends_does_not() {
        // Points along x that step back once, as scatter across a near-tangent band does, and
        // points round a quarter circle.
        let zigzag: Vec<Vec3> = [0.0, 1.0, 2.0, 1.9, 3.0, 4.0]
            .iter()
            .enumerate()
            .map(|(i, &x)| Vec3::new(x, 0.01 * i as f64, 0.0))
            .collect();
        let bend: Vec<Vec3> = (0..=8)
            .map(|k| {
                let a = std::f64::consts::FRAC_PI_2 * k as f64 / 8.0;
                Vec3::new(a.cos(), a.sin(), 0.0)
            })
            .collect();
        for (pts, folds) in [(zigzag, true), (bend, false)] {
            let c = fitted(&pts);
            let knots = c.spline().unwrap().knots();
            let range = (knots[0], knots[knots.len() - 1]);
            assert_eq!(fold(&c, range).is_some(), folds);
        }
    }
}
