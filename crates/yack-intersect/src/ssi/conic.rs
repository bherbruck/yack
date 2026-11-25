// yack: A plane against a cone whose axis is not square to it: the lines through the apex each meet the
// Adapted from springrep@a03e8a1 src/boolean/ssi/conic.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from boolean/ssi/conic.rs to intersect/ssi/conic.rs (M2.1, T2); paths only; then to yack-intersect (M3 W4), paths and visibility only.
// M3 hardening: the section's set-up as a type, its three cases (through the apex, an ellipse,
// open branches) as its methods, the region's corners shared; the same arithmetic in the same
// order.
//! A plane against a cone whose axis is not square to it: the lines through the apex each meet the
//! plane once, at `s = h / (n·g(θ))` along the generator `g(θ)`, `h` the plane's distance from the
//! apex. Where `n·g` keeps its sign the section is an ellipse, built exactly from its vertices;
//! where it changes sign the branches run out to infinity, and the parts within reach of the region
//! are evaluated along the generators and fitted. A plane through the apex holds the generators it
//! contains, touches the cone along one, or meets it at the apex alone.

use std::f64::consts::{PI, TAU};

use super::{Ssi, SsiCurve, corners, line_curve};
use crate::{
    geom::{Cone, Curve, Ellipse, IntersectionCurve, Plane, Surface},
    math::{Aabb, Frame, Vec3},
    tol,
};

/// The samples each open branch is fitted through.
const BRANCH_SAMPLES: usize = 32;

/// A plane against a cone, laid out about the apex: the generator at `θ` is
/// `g(θ) = |sin α|·(cos θ·e1 + sin θ·e2) + cos α·z`, and the plane's normal `n` meets it as
/// `n·g(θ) = a·cos θ + b`, `a ≥ 0`.
struct Section {
    apex: Vec3,
    n: Vec3,
    z: Vec3,
    e1: Vec3,
    e2: Vec3,
    /// The half angle's sine and cosine.
    sa: f64,
    ca: f64,
    a: f64,
    b: f64,
    /// The plane's distance from the apex along `n`.
    h: f64,
}

impl Section {
    /// The section of the cone `c` by the plane `p`; `None` for a cone that is a cylinder, or a
    /// plane square to its axis.
    fn new(p: &Plane, c: &Cone) -> Option<Section> {
        let (sa, ca) = c.half_angle.sin_cos();
        if sa.abs() <= tol::ANGULAR {
            return None;
        }
        let z = c.frame.z;
        let apex = c.frame.origin - z * (c.radius * ca / sa);
        let n = p.frame.z;
        let nz = n.dot(z);
        let perp = n - z * nz;
        let na = perp.norm();
        if na <= tol::ANGULAR {
            return None;
        }
        // e1 such that n·g(θ) = a·cos θ + b with a ≥ 0.
        let e1 = perp * (1.0 / na);
        let e2 = z.cross(e1);
        let (a, b) = (sa.abs() * na, ca * nz);
        let h = (p.frame.origin - apex).dot(n);
        Some(Section { apex, n, z, e1, e2, sa, ca, a, b, h })
    }

    /// The generator at `θ`.
    fn g(&self, t: f64) -> Vec3 {
        (self.e1 * t.cos() + self.e2 * t.sin()) * self.sa.abs() + self.z * self.ca
    }

    /// The generator at `θ`'s meeting with the plane.
    fn at(&self, t: f64) -> Vec3 {
        self.apex + self.g(t) * (self.h / (self.a * t.cos() + self.b))
    }

    /// Whether the point `s` along a generator lies on the cone's own nappe, the one its
    /// parameters run on, where `s·sign(sin α) > 0`; the other nappe is no part of any face.
    fn own(&self, s: f64) -> bool {
        s * self.sa > 0.0
    }

    /// A plane through the apex. The generator nearest the plane leaves it at the sine `|b| - a`:
    /// past the angular tolerance the plane meets the cone at the apex alone, within it it lies
    /// along that generator, and short of it it holds the two generators where `n·g = 0`.
    fn through_apex(&self, region: &Aabb) -> Option<Ssi> {
        let (n, a, b) = (self.n, self.a, self.b);
        let on = self.apex + n * (0.5 * self.h);
        let w = b.abs() - a;
        // Each generator from the apex onto the cone's own nappe only.
        let along = |t: f64| {
            let d = self.g(t) * self.sa.signum();
            (d - n * d.dot(n)).unit()
        };
        let ray = |d: Vec3, tangent: bool| {
            let mut c = line_curve(on, d, region, tangent)?;
            c.range.0 = c.range.0.max(0.0);
            (c.range.1 > c.range.0).then_some(c)
        };
        if w > tol::ANGULAR {
            return Some(Ssi::Curves { curves: Vec::new(), points: vec![on] });
        }
        if w >= -tol::ANGULAR {
            let d = along(if b < 0.0 { 0.0 } else { PI })?;
            return Some(Ssi::curves(ray(d, true).into_iter().collect()));
        }
        let t = (-b / a).acos();
        let curves = [t, -t].into_iter().filter_map(|t| ray(along(t)?, false)).collect();
        Some(Ssi::Curves { curves, points: vec![on] })
    }

    /// The section where every generator meets the plane on one side of the apex: an ellipse,
    /// whose long axis lies in the plane through the axis and the normal, from θ = 0 to θ = π.
    fn ellipse(&self) -> Option<Ssi> {
        let (p0, p1) = (self.at(0.0), self.at(PI));
        let centre = p0.lerp(p1, 0.5);
        let half = (p0 - p1) * 0.5;
        let major = half.norm();
        let x = half * (1.0 / major);
        let y = self.n.cross(x);
        let q = self.at(0.5 * PI) - centre;
        let (qx, qy) = (q.dot(x) / major, q.dot(y));
        let minor = qy.abs() / (1.0 - qx * qx).max(0.0).sqrt();
        if !minor.is_finite() || minor <= 0.0 {
            return None;
        }
        let frame = Frame { origin: centre, x, y, z: x.cross(y) };
        Some(Ssi::curves(vec![SsiCurve {
            curve: Curve::Ellipse(Ellipse { frame, major, minor }),
            range: (0.0, TAU),
            closed: true,
            tangent: false,
        }]))
    }

    /// The branches out to infinity, each over the angles where `n·g` keeps one sign, and the
    /// part of it within `reach` of the apex over those where `|n·g| ≥ |h| / reach`, fitted.
    fn branches(&self, p: &Plane, c: &Cone, reach: f64) -> Option<Ssi> {
        let (a, b, h) = (self.a, self.b, self.h);
        let k = h.abs() / reach;
        let mut arcs: Vec<(f64, f64)> = Vec::new();
        // a·cos θ + b ≥ k about θ = 0, and ≤ -k about θ = π.
        let c_hi = (k - b) / a;
        if c_hi < 1.0 && self.own(h) {
            let t = c_hi.max(-1.0).acos();
            arcs.push((-t, t));
        }
        let c_lo = (-k - b) / a;
        if c_lo > -1.0 && self.own(-h) {
            let t = c_lo.min(1.0).acos();
            arcs.push((t, TAU - t));
        }
        let (sp, sc) = (Surface::Plane(p.clone()), Surface::Cone(c.clone()));
        let mut curves = Vec::new();
        for (t0, t1) in arcs {
            let sample = |s: f64| self.at(t0 + (t1 - t0) * s);
            let fit = IntersectionCurve::fit_sampled(
                &sample,
                (0.0, 1.0),
                sp.clone(),
                sc.clone(),
                false,
                BRANCH_SAMPLES,
            )
            .ok()?;
            let range = fit.approx.domain();
            curves.push(SsiCurve {
                curve: Curve::Intersection(Box::new(fit)),
                range,
                closed: false,
                tangent: false,
            });
        }
        Some(Ssi::curves(curves))
    }
}

/// The section of the cone `c` by the plane `p`, within `region`; `None` where it has no closed
/// form here.
pub(crate) fn plane_cone(p: &Plane, c: &Cone, region: &Aabb) -> Option<Ssi> {
    let section = Section::new(p, c)?;
    let (a, b, h) = (section.a, section.b, section.h);
    if h.abs() <= tol::LINEAR {
        return section.through_apex(region);
    }
    // The farthest the region reaches from the apex.
    let reach = corners(region).map(|corner| corner.dist(section.apex)).fold(0.0, f64::max);
    // An ellipse wholly within reach is built exactly; one that is not (a plane all but parallel to
    // a generator) is followed, as a parabola or hyperbola is, only as far as the region reaches.
    if b.abs() > a && h.abs() <= reach * (b.abs() - a) {
        if !section.own(h / (a + b)) {
            return Some(Ssi::none());
        }
        return section.ellipse();
    }
    section.branches(p, c, reach)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{csi::signed_distance, math::Point3};

    /// A cone about z with its apex at the origin, opening upward at 45°.
    fn cone() -> Cone {
        Cone {
            frame: Frame::new(Vec3::new(0.0, 0.0, 1.0), Vec3::Z, Vec3::X).unwrap(),
            radius: 1.0,
            half_angle: PI / 4.0,
        }
    }

    fn plane(o: [f64; 3], n: [f64; 3]) -> Plane {
        Plane {
            frame: Frame::from_axis(Vec3::from_array(o), Vec3::from_array(n).unit().unwrap())
                .unwrap(),
        }
    }

    fn region() -> Aabb {
        Aabb { min: Vec3::new(-10.0, -10.0, -10.0), max: Vec3::new(10.0, 10.0, 10.0) }
    }

    fn on_both(p: &Plane, c: &Cone, cs: &[SsiCurve]) {
        let (sp, sc) = (Surface::Plane(p.clone()), Surface::Cone(c.clone()));
        for cv in cs {
            for i in 0..=64 {
                let x = cv.curve.eval(cv.range.0 + (cv.range.1 - cv.range.0) * i as f64 / 64.0);
                if (0..3).any(|k| x[k].abs() > 10.0) {
                    continue;
                }
                for s in [&sp, &sc] {
                    let d = signed_distance(s, x).abs();
                    assert!(d <= tol::LINEAR, "{} off {} by {d}", cv.curve.kind(), s.kind());
                }
            }
        }
    }

    fn meet(p: &Plane, c: &Cone) -> (Vec<SsiCurve>, Vec<Point3>) {
        let Some(Ssi::Curves { curves, points }) = plane_cone(p, c, &region()) else {
            panic!("no closed form");
        };
        on_both(p, c, &curves);
        (curves, points)
    }

    #[test]
    fn a_narrowing_cone_keeps_the_branch_below_its_apex() {
        // r 4 at z 0 narrowing at 45° to its apex at (0, 0, 4): y = 2 cuts x² + 4 = (4 - z)², whose
        // branch on the cone's own nappe passes (0, 2, 2).
        let c = Cone { frame: Frame::WORLD, radius: 4.0, half_angle: -PI / 4.0 };
        let (cs, pts) = meet(&plane([0.0, 2.0, 0.0], [0.0, 1.0, 0.0]), &c);
        assert!(pts.is_empty() && cs.len() == 1);
        let at = Vec3::new(0.0, 2.0, 2.0);
        let (d, _) = crate::csi::curve_distance(&cs[0].curve, cs[0].range, at);
        assert!(d < 1e-9, "{d}");
    }

    #[test]
    fn a_plane_tilted_less_than_the_cone_cuts_an_ellipse() {
        // z = 2 + 0.5·x cuts the 45° cone in an ellipse whose vertices lie in y = 0, where
        // |x| = z: x = 2 + 0.5x gives (4, 0, 4), and -x = 2 + 0.5x gives (-4/3, 0, 4/3).
        let c = cone();
        let p = plane([0.0, 0.0, 2.0], [-0.5, 0.0, 1.0]);
        let (cs, pts) = meet(&p, &c);
        assert!(pts.is_empty() && cs.len() == 1);
        let Curve::Ellipse(e) = &cs[0].curve else {
            panic!("not an ellipse");
        };
        let half = 0.5 * Vec3::new(4.0, 0.0, 4.0).dist(Vec3::new(-4.0 / 3.0, 0.0, 4.0 / 3.0));
        assert!((e.major - half).abs() < 1e-12, "{}", e.major);
    }

    #[test]
    fn a_plane_steeper_than_the_cone_cuts_a_hyperbola_or_parabola() {
        // x = 1 is parallel to the axis: a hyperbola z² - y² = 1, whose branch on the cone's own
        // nappe (z > 0) is kept.
        let c = cone();
        let (cs, pts) = meet(&plane([1.0, 0.0, 0.0], [1.0, 0.0, 0.0]), &c);
        assert!(pts.is_empty() && cs.len() == 1 && !cs[0].closed);
        assert!(cs[0].curve.eval(cs[0].range.0).z > 0.0);
        // z = 1 + x is parallel to a generator: one parabola. Tilted 1e-12 further it is an
        // ellipse some 1e12 long, followed as far as the region reaches as the parabola is.
        let (cs, _) = meet(&plane([0.0, 0.0, 1.0], [-1.0, 0.0, 1.0]), &c);
        assert_eq!(cs.len(), 1);
        let (cs, _) = meet(&plane([0.0, 0.0, 1.0], [-1.0, 0.0, 1.0 + 1e-12]), &c);
        assert!(cs.len() == 1 && !cs[0].closed);
        let ends = [cs[0].curve.eval(cs[0].range.0), cs[0].curve.eval(cs[0].range.1)];
        assert!(ends.iter().all(|p| p.norm() < 100.0), "{ends:?}");
    }

    #[test]
    fn a_plane_through_the_apex_holds_two_generators_touches_along_one_or_meets_the_apex() {
        let c = cone();
        let (cs, pts) = meet(&plane([0.0; 3], [1.0, 0.0, 0.0]), &c);
        assert!(cs.len() == 2 && pts.len() == 1 && cs.iter().all(|cv| !cv.tangent));
        let (cs, pts) = meet(&plane([0.0; 3], [-1.0, 0.0, 1.0]), &c);
        assert!(cs.len() == 1 && cs[0].tangent && pts.is_empty());
        let (cs, pts) = meet(&plane([0.0; 3], [-0.5, 0.0, 1.0]), &c);
        assert!(cs.is_empty() && pts.len() == 1 && pts[0].norm() < 1e-12);
    }
}
