// yack: a plane against a plane, a cylinder or a sphere, in closed form.
// Adapted from springrep@a03e8a1 src/boolean/ssi.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: split out of ssi.rs (M3 hardening): each declines (`None`) where it cannot build its
// curve's frame or direction, instead of panicking; the region's corners shared; the same
// arithmetic in the same order.
//! A plane against a plane (a line), a cylinder (a circle, two lines, a touching line or an
//! ellipse) or a sphere (a circle or a touching point). Each is `None` only where a direction it
//! needs has no length, which the tests before it rule out.

use std::f64::consts::TAU;

use super::{Ssi, SsiCurve, circle_curve, corners, frame_at, line_curve, parallel};
use crate::{
    geom::{Curve, Cylinder, Ellipse, Sphere},
    math::{Aabb, Frame, Point3},
    tol,
};

/// How far off lying along a plane, in radians, a cylinder's axis may be for [`nearly_lines`] to
/// try lines for their crossing.
const NEARLY_ALONG: f64 = 1e-6; // tol: an angle, in radians

/// A cylinder's points on the plane are a pair only while the cosine that places them is this far
/// inside ±1.
const PAIR_MARGIN: f64 = 1e-9; // tol: a cosine's rounding

/// Two planes meet in a line, unless they are parallel.
pub(super) fn plane_plane(a: &Frame, b: &Frame, region: &Aabb) -> Option<Ssi> {
    let (n1, n2) = (a.z, b.z);
    let d = n1.cross(n2);
    if d.norm() <= tol::ANGULAR {
        return Some(Ssi::none());
    }
    let (h1, h2) = (n1.dot(a.origin), n2.dot(b.origin));
    // n1·(h1 − h2·c)/(1 − c²) + n2·(h2 − h1·c)/(1 − c²), arranged so planes a hair off parallel do
    // not take the difference of two large numbers: at an angle θ, 1 − c² is θ², and that form's
    // rounding put the point θ⁻² · 1e-16 off both planes, 1e-5 mm at θ = 1.5e-5.
    let p0 = (n2 * h1 - n1 * h2).cross(d) * (1.0 / d.dot(d));
    let dir = d.unit()?;
    Some(Ssi::curves(line_curve(p0, dir, region, false).into_iter().collect()))
}

/// A plane against a cylinder: a circle across the axis, lines (or a touching line) along it, an
/// ellipse otherwise.
pub(super) fn plane_cylinder(p: &Frame, c: &Cylinder, region: &Aabb) -> Option<Ssi> {
    let (n, a, r) = (p.z, c.frame.z, c.radius);
    let an = a.dot(n);
    if parallel(a, n) {
        let h = (p.origin - c.frame.origin).dot(a);
        let fr = Frame { origin: c.frame.origin + a * h, ..c.frame };
        return Some(Ssi::curves(vec![circle_curve(fr, r, false)]));
    }
    if an.abs() <= tol::ANGULAR {
        let delta = (c.frame.origin - p.origin).dot(n);
        let foot = c.frame.origin - n * delta;
        if delta.abs() > r + tol::LINEAR {
            return Some(Ssi::none());
        }
        let curves = if delta.abs() >= r - tol::LINEAR {
            // Midway between the plane and the cylinder's nearest line, so within the tolerance
            // of both however the gap falls in the band.
            let mid = c.frame.origin - n * (0.5 * (delta + delta.signum() * r));
            line_curve(mid, a, region, true).into_iter().collect()
        } else {
            let w = a.cross(n).unit()?;
            let s = ((r - delta.abs()) * (r + delta.abs())).sqrt();
            [foot + w * s, foot - w * s]
                .into_iter()
                .filter_map(|q| line_curve(q, a, region, false))
                .collect()
        };
        return Some(Ssi::curves(curves));
    }
    if an.abs() <= NEARLY_ALONG
        && let Some(curves) = nearly_lines(p, c, region)
    {
        return Some(Ssi::curves(curves));
    }
    let t = (p.origin - c.frame.origin).dot(n) / an;
    let centre = c.frame.origin + a * t;
    let w = a.cross(n).unit()?;
    let m = n.cross(w);
    let frame = Frame { origin: centre, x: m, y: n.cross(m), z: n };
    let ellipse = Curve::Ellipse(Ellipse { frame, major: r / an.abs(), minor: r });
    Some(Ssi::curves(vec![SsiCurve {
        curve: ellipse,
        range: (0.0, TAU),
        closed: true,
        tangent: false,
    }]))
}

/// The plane `p` across the cylinder `c` whose axis lies a hair off along it, as two lines through
/// `region`, where those lines lie within a tenth of the tolerance of the crossing: an axis turned
/// 6e-10 off by a rounded pose makes it an ellipse 2.8e9 mm long for a radius of 1.6, whose centre,
/// 1e9 mm away, rounds its points by the tolerance. `None` where they do not, or where the plane
/// touches the cylinder or misses it somewhere across the region.
fn nearly_lines(p: &Frame, c: &Cylinder, region: &Aabb) -> Option<Vec<SsiCurve>> {
    let (n, a, r) = (p.z, c.frame.z, c.radius);
    let o = c.frame.origin;
    let (ax, ay) = (c.frame.x.dot(n), c.frame.y.dot(n));
    let rho = ax.hypot(ay);
    let phase = ay.atan2(ax);
    let lift = (o - p.origin).dot(n);
    let (lo, hi) = corners(region).fold((f64::MAX, f64::MIN), |(lo, hi), q| {
        let t = (q - o).dot(a);
        (lo.min(t), hi.max(t))
    });
    if lo >= hi {
        return None;
    }
    // Where along the axis at `t` the cylinder's two points on the plane are: the angle round it
    // whose cosine against the plane's normal makes up the rest of the plane's equation.
    let at = |t: f64, side: f64| -> Option<Point3> {
        let k = -(lift + a.dot(n) * t) / (r * rho);
        if k.abs() >= 1.0 - PAIR_MARGIN {
            return None;
        }
        let theta = phase + side * k.acos();
        Some(o + a * t + (c.frame.x * theta.cos() + c.frame.y * theta.sin()) * r)
    };
    let mut out = Vec::new();
    for side in [1.0, -1.0] {
        let (q0, q1, qm) = (at(lo, side)?, at(hi, side)?, at(0.5 * (lo + hi), side)?);
        let dir = (q1 - q0).unit()?;
        let off = (qm - q0) - dir * (qm - q0).dot(dir);
        if off.norm() > 0.1 * tol::LINEAR {
            return None;
        }
        out.extend(line_curve(q0, dir, region, false));
    }
    Some(out)
}

/// A plane against a sphere: a circle, a touching point, or nothing.
pub(super) fn plane_sphere(p: &Frame, s: &Sphere) -> Option<Ssi> {
    let n = p.z;
    let delta = (s.frame.origin - p.origin).dot(n);
    let r = s.radius;
    let foot = s.frame.origin - n * delta;
    Some(if delta.abs() > r + tol::LINEAR {
        Ssi::none()
    } else if delta.abs() >= r - tol::LINEAR {
        let touch = s.frame.origin - n * (0.5 * (delta + delta.signum() * r));
        Ssi::Curves { curves: Vec::new(), points: vec![touch] }
    } else {
        let rho = ((r - delta.abs()) * (r + delta.abs())).sqrt();
        Ssi::curves(vec![circle_curve(frame_at(foot, n, s.frame.x)?, rho, false)])
    })
}
