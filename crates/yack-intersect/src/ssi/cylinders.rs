// yack: two cylinders in closed form: parallel ones in lines, equal crossing ones in two ellipses.
// Adapted from springrep@a03e8a1 src/boolean/ssi.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: split out of ssi.rs (M3 hardening): the equal crossing pair declines (`None`) where its
// axes give no plane instead of panicking; the same arithmetic in the same order.
//! Two cylinders in closed form: parallel ones meet in lines along their axes, and two of one
//! radius whose axes cross meet in two ellipses. Every other pair is the generators' sweep's.

use std::f64::consts::TAU;

use super::{Ssi, SsiCurve, line_curve, parallel};
use crate::{
    geom::{Curve, Cylinder, Ellipse},
    math::{Aabb, Frame, Point3, Vec3},
    tol,
};

/// Two cylinders of one radius whose axes cross meet in two ellipses, in the planes bisecting
/// the axes, which touch where the common perpendicular meets the cylinders.
#[allow(clippy::panic)]
fn equal_crossing_cylinders(o: Point3, a1: Vec3, a2: Vec3, r: f64) -> Option<Ssi> {
    let nhat = a1.cross(a2).unit().unwrap_or_else(|| panic!("axes are not parallel"));
    let mut curves = Vec::new();
    for m in [a1 - a2, a1 + a2] {
        let m = m.unit().unwrap_or_else(|| panic!("axes are not parallel"));
        let e = m.cross(nhat);
        let frame = Frame { origin: o, x: e, y: m.cross(e), z: m };
        curves.push(SsiCurve {
            curve: Curve::Ellipse(Ellipse { frame, major: r / e.cross(a1).norm(), minor: r }),
            range: (0.0, TAU),
            closed: true,
            tangent: false,
        });
    }
    Some(Ssi::Curves { curves, points: vec![o + nhat * r, o - nhat * r] })
}

/// The closest points of two lines, as parameters along each; `None` when they are parallel.
fn closest(o1: Point3, d1: Vec3, o2: Point3, d2: Vec3) -> Option<(f64, f64)> {
    let w = o1 - o2;
    let c = d1.dot(d2);
    let den = 1.0 - c * c;
    if den.sqrt() <= tol::ANGULAR {
        return None;
    }
    let (d, e) = (d1.dot(w), d2.dot(w));
    Some(((c * e - d) / den, (e - c * d) / den))
}

/// Two cylinders whose axes are parallel, or which have one radius and axes that cross; `None` for
/// any other pair.
pub(super) fn cylinder_cylinder(a: &Cylinder, b: &Cylinder, region: &Aabb) -> Option<Ssi> {
    let (a1, a2) = (a.frame.z, b.frame.z);
    if parallel(a1, a2) {
        // Parallel axes: the circles of a cross-section meet at points along the axis.
        let w = b.frame.origin - a.frame.origin;
        let off = w - a1 * w.dot(a1);
        let d = off.norm();
        let (ra, rb) = (a.radius, b.radius);
        if d > ra + rb + tol::LINEAR || d < (ra - rb).abs() - tol::LINEAR {
            return Some(Ssi::none());
        }
        let ex = off / d;
        let ey = a1.cross(ex);
        let x = (d * d + ra * ra - rb * rb) / (2.0 * d);
        let touching =
            (d - (ra + rb)).abs() <= tol::LINEAR || (d - (ra - rb).abs()).abs() <= tol::LINEAR;
        let feet: Vec<Point3> = if touching {
            vec![a.frame.origin + ex * x]
        } else {
            let y = (ra * ra - x * x).max(0.0).sqrt();
            vec![a.frame.origin + ex * x + ey * y, a.frame.origin + ex * x - ey * y]
        };
        let curves = feet.into_iter().filter_map(|q| line_curve(q, a1, region, touching)).collect();
        return Some(Ssi::curves(curves));
    }
    if (a.radius - b.radius).abs() <= tol::LINEAR {
        let (s, t) = closest(a.frame.origin, a1, b.frame.origin, a2)?;
        let (p, q) = (a.frame.origin + a1 * s, b.frame.origin + a2 * t);
        // About the axes' midpoint with the mean radius, the ellipses lie within half the radii's
        // and the axes' gaps of each surface: within half the tolerance, which their meetings with
        // other curves need on top of it. Past that the generators sweep (`sweep.rs`).
        if p.dist(q) + (a.radius - b.radius).abs() <= tol::LINEAR {
            let r = 0.5 * (a.radius + b.radius);
            return equal_crossing_cylinders(p.lerp(q, 0.5), a1, a2, r);
        }
    }
    None
}
