// yack: circles and ellipses against planes, cylinders, spheres and lines, in closed form.
// Adapted from springrep@a03e8a1 src/boolean/csi.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: split out of csi.rs (M3 hardening): the plane cases out of the dispatcher, and a line
// against a conic given the conic's plane instead of panicking without one; the same arithmetic in
// the same order.
//! Circles and ellipses against planes, cylinders square to them, spheres and lines in their own
//! plane, all in closed form: a conic's point is a sum of a cosine and a sine of its angle.

use std::f64::consts::TAU;

use super::{
    Hits, curve_distance, curve_surface, in_range,
    line::line_cylinder,
    roots::{angle_in, trig_roots},
};
use crate::{
    geom::{Circle, Curve, Cylinder, Ellipse, Plane, Sphere, Surface},
    math::{Frame, Point3, Vec2},
    tol,
};

/// A circle against a plane, over the angle range `range`.
pub(super) fn circle_plane(ci: &Circle, p: &Plane, range: (f64, f64)) -> Hits {
    let n = p.frame.z;
    let a = (ci.frame.origin - p.frame.origin).dot(n);
    trig_roots(a, ci.radius * ci.frame.x.dot(n), ci.radius * ci.frame.y.dot(n), range)
}

/// An ellipse against a plane, over the angle range `range`.
pub(super) fn ellipse_plane(e: &Ellipse, p: &Plane, range: (f64, f64)) -> Hits {
    let n = p.frame.z;
    let a = (e.frame.origin - p.frame.origin).dot(n);
    trig_roots(a, e.major * e.frame.x.dot(n), e.minor * e.frame.y.dot(n), range)
}

/// A circle square to a cylinder's axis meets it where it meets the cylinder's section in its own
/// plane; a fillet's tangent contact sampled numerically would split into two nearby crossings.
pub(super) fn circle_parallel_cylinder(ci: &Circle, cyl: &Cylinder, range: (f64, f64)) -> Hits {
    let l = ci.frame.to_local(cyl.frame.origin);
    coplanar_circles(ci, Vec2::new(l.x, l.y), cyl.radius, range)
}

/// A circle against another of radius `big` centred at `centre` in its plane (local coordinates).
fn coplanar_circles(ci: &Circle, centre: Vec2, big: f64, range: (f64, f64)) -> Hits {
    let d = centre.x.hypot(centre.y);
    let r = ci.radius;
    if d <= tol::LINEAR {
        return Hits { ts: Vec::new(), on_surface: (r - big).abs() <= tol::LINEAR };
    }
    if d > r + big + tol::LINEAR || d < (r - big).abs() - tol::LINEAR {
        return Hits::default();
    }
    let toward = centre.y.atan2(centre.x);
    let x = (d * d + r * r - big * big) / (2.0 * d);
    let touching =
        (d - (r + big)).abs() <= tol::LINEAR || (d - (r - big).abs()).abs() <= tol::LINEAR;
    let angles = if touching {
        vec![if x >= 0.0 { toward } else { toward + TAU / 2.0 }]
    } else {
        let a = (x / r).clamp(-1.0, 1.0).acos();
        vec![toward - a, toward + a]
    };
    Hits { ts: angles.into_iter().filter_map(|a| angle_in(a, range)).collect(), on_surface: false }
}

/// A circle against a sphere meets it where it meets the sphere's section in the circle's plane.
pub(super) fn circle_sphere(ci: &Circle, sp: &Sphere, range: (f64, f64)) -> Hits {
    let l = ci.frame.to_local(sp.frame.origin);
    let rho2 = (sp.radius - l.z.abs()) * (sp.radius + l.z.abs());
    if rho2 < 0.0 {
        // The plane misses the sphere; within the tolerance it touches it at the foot.
        let gap = l.z.abs() - sp.radius;
        let d = l.x.hypot(l.y);
        return if gap <= tol::LINEAR && (d - ci.radius).abs() <= tol::LINEAR {
            Hits { ts: angle_in(l.y.atan2(l.x), range).into_iter().collect(), on_surface: false }
        } else {
            Hits::default()
        };
    }
    coplanar_circles(ci, Vec2::new(l.x, l.y), rho2.sqrt(), range)
}

/// The curve's plane, when it is a planar conic.
pub(super) fn conic_plane(c: &Curve) -> Option<Frame> {
    match c {
        Curve::Circle(ci) => Some(ci.frame),
        Curve::Ellipse(e) => Some(e.frame),
        _ => None,
    }
}

/// Where the line `line` over `rl` meets the conic `conic` over `rc`, whose plane is `frame`.
pub(super) fn line_conic(
    line: &Curve,
    rl: (f64, f64),
    conic: &Curve,
    rc: (f64, f64),
    frame: Frame,
) -> Vec<Point3> {
    let Curve::Line(l) = line else {
        return Vec::new();
    };
    let in_plane = l.dir.dot(frame.z).abs() <= tol::ANGULAR
        && (l.origin - frame.origin).dot(frame.z).abs() <= tol::LINEAR;
    let on_conic = |p: Point3| curve_distance(conic, rc, p).0 <= tol::LINEAR;
    if !in_plane {
        let plane = Surface::Plane(Plane { frame });
        return curve_surface(line, rl, &plane)
            .ts
            .into_iter()
            .map(|t| line.eval(t))
            .filter(|&p| on_conic(p))
            .collect();
    }
    let ts = match conic {
        Curve::Circle(ci) => {
            line_cylinder(l.origin, l.dir, &Cylinder { frame: ci.frame, radius: ci.radius }).ts
        }
        Curve::Ellipse(e) => {
            // In the ellipse's frame scaled to a unit circle, the line stays a line.
            let lo = e.frame.to_local(l.origin);
            let ld = e.frame.dir_to_local(l.dir);
            let (ox, oy) = (lo.x / e.major, lo.y / e.minor);
            let (dx, dy) = (ld.x / e.major, ld.y / e.minor);
            let a = dx * dx + dy * dy;
            let t_near = -(ox * dx + oy * dy) / a;
            let disc = 1.0 - ((ox + dx * t_near).powi(2) + (oy + dy * t_near).powi(2));
            let near = line.eval(t_near);
            if on_conic(near) {
                vec![t_near]
            } else if disc > 0.0 {
                let h = (disc / a).sqrt();
                vec![t_near - h, t_near + h]
            } else {
                Vec::new()
            }
        }
        _ => return Vec::new(),
    };
    ts.into_iter()
        .filter_map(|t| in_range(line, t, rl))
        .map(|t| line.eval(t))
        .filter(|&p| on_conic(p))
        .collect()
}
