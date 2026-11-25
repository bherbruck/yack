// yack: surfaces of revolution about one axis, which meet in circles about it.
// Adapted from springrep@a03e8a1 src/boolean/ssi.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: split out of ssi.rs (M3 hardening): meridians in the half plane as `Vec2`s instead of
// two-element arrays, circles that decline (`None`) where their frame cannot be built instead of
// panicking; the same arithmetic in the same order.
//! Surfaces of revolution about one axis: each is its meridian, a line or a circle in the half
//! plane `(ρ, z)` about the axis, and two such surfaces meet in the circles their meridians' common
//! points sweep.

use super::{Axis, Ssi, SsiCurve, circle_curve, frame_at};
use crate::{
    geom::Surface,
    math::{Point3, Vec2, Vec3},
    tol,
};

/// A meridian of a surface of revolution, in the half plane `(ρ, z)` about an axis.
#[derive(Clone, Copy, Debug)]
pub(super) enum Meridian {
    /// Through `p` along the unit `d`.
    Line {
        p: Vec2,
        d: Vec2,
    },
    Circle {
        c: Vec2,
        r: f64,
    },
}

/// The meridian of a cylinder, cone, sphere or torus about `axis`.
pub(super) fn meridian(s: &Surface, axis: Axis) -> Option<Meridian> {
    let z_of = |p: Point3| (p - axis.origin).dot(axis.dir);
    match s {
        Surface::Cylinder(c) => Some(Meridian::Line {
            p: Vec2::new(c.radius, z_of(c.frame.origin)),
            d: Vec2::new(0.0, 1.0),
        }),
        Surface::Cone(c) => {
            let (sa, ca) = c.half_angle.sin_cos();
            let flip = if c.frame.z.dot(axis.dir) > 0.0 { 1.0 } else { -1.0 };
            Some(Meridian::Line {
                p: Vec2::new(c.radius, z_of(c.frame.origin)),
                d: Vec2::new(sa, ca * flip),
            })
        }
        Surface::Sphere(sp) => {
            Some(Meridian::Circle { c: Vec2::new(0.0, z_of(sp.frame.origin)), r: sp.radius })
        }
        Surface::Torus(t) => {
            Some(Meridian::Circle { c: Vec2::new(t.major, z_of(t.frame.origin)), r: t.minor })
        }
        _ => None,
    }
}

/// Where a meridian line through `p` along the unit `d` meets the circle about `c` of radius `r`,
/// each point with whether they touch there.
fn line_circle(p: Vec2, d: Vec2, c: Vec2, r: f64) -> Vec<(Vec2, bool)> {
    let w = p - c;
    let t_near = -w.dot(d);
    let q = p + d * t_near;
    let m = q - c;
    let m = m.x.hypot(m.y);
    if m > r + tol::LINEAR {
        Vec::new()
    } else if m >= r - tol::LINEAR {
        vec![(q, true)]
    } else {
        let h = ((r - m) * (r + m)).sqrt();
        vec![(q + d * h, false), (q - d * h, false)]
    }
}

/// The meridians' common points, each with whether they touch there; `None` when they coincide.
pub(super) fn meridian_points(a: Meridian, b: Meridian) -> Option<Vec<(Vec2, bool)>> {
    match (a, b) {
        (Meridian::Line { p: p1, d: d1 }, Meridian::Line { p: p2, d: d2 }) => {
            let den = d1.cross(d2);
            let w = p2 - p1;
            if den.abs() <= tol::ANGULAR {
                return if w.cross(d1).abs() <= tol::LINEAR { None } else { Some(Vec::new()) };
            }
            let t = w.cross(d2) / den;
            Some(vec![(p1 + d1 * t, false)])
        }
        (Meridian::Line { p, d }, Meridian::Circle { c, r })
        | (Meridian::Circle { c, r }, Meridian::Line { p, d }) => Some(line_circle(p, d, c, r)),
        (Meridian::Circle { c: c1, r: r1 }, Meridian::Circle { c: c2, r: r2 }) => {
            let w = c2 - c1;
            let d = w.x.hypot(w.y);
            if d <= tol::LINEAR {
                return if (r1 - r2).abs() <= tol::LINEAR { None } else { Some(Vec::new()) };
            }
            if d > r1 + r2 + tol::LINEAR || d < (r1 - r2).abs() - tol::LINEAR {
                return Some(Vec::new());
            }
            let ex = w / d;
            let ey = ex.perp();
            let x = (d * d + r1 * r1 - r2 * r2) / (2.0 * d);
            let touching =
                (d - (r1 + r2)).abs() <= tol::LINEAR || (d - (r1 - r2).abs()).abs() <= tol::LINEAR;
            if touching {
                return Some(vec![(c1 + ex * x, true)]);
            }
            let y = (r1 * r1 - x * x).max(0.0).sqrt();
            Some(vec![(c1 + ex * x + ey * y, false), (c1 + ex * x - ey * y, false)])
        }
    }
}

/// Two surfaces of revolution about one axis meet in circles about it.
pub(super) fn coaxial(a: &Surface, b: &Surface, axis: Axis) -> Option<Ssi> {
    let (ma, mb) = (meridian(a, axis)?, meridian(b, axis)?);
    let Some(pts) = meridian_points(ma, mb) else {
        return Some(Ssi::Coincident);
    };
    let x_hint = match a {
        Surface::Cylinder(c) => c.frame.x,
        Surface::Cone(c) => c.frame.x,
        Surface::Torus(t) => t.frame.x,
        Surface::Sphere(s) => s.frame.x,
        _ => axis.dir.any_perpendicular(),
    };
    circles_about(axis, x_hint, pts)
}

/// The circles about `axis` through meridian points; a point on the axis is a single point, and a
/// point at negative `ρ` (a cone's other nappe) is the circle of its distance.
pub(super) fn circles_about(axis: Axis, x_hint: Vec3, pts: Vec<(Vec2, bool)>) -> Option<Ssi> {
    let mut curves: Vec<SsiCurve> = Vec::new();
    let mut points = Vec::new();
    let mut seen: Vec<(f64, f64)> = Vec::new();
    for (at, touching) in pts {
        let (rho, z) = (at.x.abs(), at.y);
        if seen.iter().any(|&(r, h)| (r - rho).abs() <= tol::LINEAR && (h - z).abs() <= tol::LINEAR)
        {
            continue;
        }
        seen.push((rho, z));
        let centre = axis.origin + axis.dir * z;
        if rho <= tol::LINEAR {
            points.push(centre);
        } else {
            curves.push(circle_curve(frame_at(centre, axis.dir, x_hint)?, rho, touching));
        }
    }
    Some(Ssi::Curves { curves, points })
}
