// yack: the solid a cylinder, cone or sphere surface encloses, for closing a deleted gap against it
//! A face on a cylinder, a cone or a sphere bounds the solid its surface encloses: the prism of the
//! cylinder, the nappe of the cone, the ball. Closing a gap against such a face fills or cuts to
//! that body, or to its complement where the face is a hole's (its normal turned inward).

use crate::{
    Error, Result,
    geom::Surface,
    math::{Frame, Point3, Vec2},
    measure,
    profile::Profile,
    sweep::extrude,
    tol,
    topo::{Brep, FaceId},
};

/// Whether `q` lies in the body `surface` encloses; `None` for a surface that encloses none here.
pub(crate) fn within(surface: &Surface, q: Point3) -> Option<bool> {
    match surface {
        Surface::Cylinder(c) => {
            let off = q - c.frame.origin;
            Some((off - c.frame.z * off.dot(c.frame.z)).norm() <= c.radius)
        }
        Surface::Cone(c) => {
            let off = q - c.frame.origin;
            let h = off.dot(c.frame.z);
            let reach = c.radius + h * c.half_angle.tan();
            Some(reach >= 0.0 && (off - c.frame.z * h).norm() <= reach)
        }
        Surface::Sphere(sp) => Some(q.dist(sp.frame.origin) <= sp.radius),
        _ => None,
    }
}

/// Whether face `f` is a hole's: its material lies outside the body its surface encloses, where
/// its outward normal leans in toward the axis (or the ball's centre). A cone's own normal leans
/// out or in with its half angle, so this is read from the normal and not the face's flag alone.
pub(crate) fn hollow(s: &Brep, f: FaceId) -> bool {
    let face = s.face(f);
    let leans_out = match &face.surface {
        Surface::Cone(c) => {
            let n = face.surface.normal(0.0, 0.0);
            n.dot(c.frame.x) > 0.0
        }
        _ => true,
    };
    face.reversed == leans_out
}

/// The body `surface` encloses, long enough to cover `s` (a cone's, to its apex).
pub(crate) fn body(s: &Brep, surface: &Surface) -> Result<Brep> {
    let b = measure::bbox(s);
    let around = |at: Point3| b.diagonal() + b.center().dist(at) + 1.0;
    match surface {
        Surface::Cylinder(c) => {
            let reach = around(c.frame.origin);
            let base = c.frame.origin - c.frame.z * reach;
            let fr = Frame::new(base, c.frame.z, c.frame.x)?;
            extrude(&Profile::circle(Vec2::ZERO, 2.0 * c.radius), &fr, 2.0 * reach, "cyl")
        }
        Surface::Cone(c) => {
            let reach = around(c.frame.origin);
            // The radius at height `h` along the axis grows by the tangent of the half angle.
            let slope = c.half_angle.tan();
            let at = |h: f64| c.radius + h * slope;
            // The nappe from `lo` to `hi` along the axis, cut where the radius runs out at the apex.
            let apex = -c.radius / slope;
            let (lo, hi) = if slope > 0.0 { (apex.max(-reach), reach) } else { (-reach, apex.min(reach)) };
            let fr = Frame::new(c.frame.origin + c.frame.z * lo, c.frame.z, c.frame.x)?;
            // A radius that has run out to within the tolerance is the apex itself.
            let radius = |h: f64| if at(h) <= tol::LINEAR { 0.0 } else { at(h) };
            yack_build::primitive::cone(&fr, radius(lo), radius(hi), hi - lo, "cone")
        }
        Surface::Sphere(sp) => yack_build::primitive::sphere(&sp.frame, sp.radius, "ball"),
        other => Err(Error::Unsupported(format!("closing a gap against a {} face", other.kind()))),
    }
}
