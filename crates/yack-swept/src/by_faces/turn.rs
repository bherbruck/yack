// yack: how faces stand to a turn about an axis, and the surfaces their edges sweep (M4.1 step 1).
//! By faces for a turn: each point moves on a circle about the axis, at the velocity
//! `axis × (p − origin)` per radian.
//!
//! Which faces this builds: those whose every leading piece meets each circle about the axis at
//! most once. That keeps the piece's revolved prism from overlapping itself however far it turns
//! (short of a whole turn): a circle crosses a plane, a sphere or a cylinder parallel to the axis
//! at most twice, the crossings alternate in and out, so a piece that only leads meets it once.
//! Their contacts are planar too:
//! - a plane with normal `n` touches where `(p − origin)·(n × axis) = 0`: the plane through the axis
//!   square to `n × axis` (a plane square to the axis turns within itself);
//! - a sphere about `c` where `(p − c)·(axis × (c − origin)) = 0`: the plane through its centre
//!   square to `axis × (c − origin)` (a sphere centred on the axis turns within itself);
//! - a cylinder parallel to the axis through `c` likewise, the plane through its own axis.
//!
//! Faces that turn within themselves (a cylinder, cone, sphere or torus about the axis itself)
//! run along. Every other face declines.

use yack_base::kernel::{Error, Result};
use yack_build::build::PartFace;
use yack_geom::{Cone, Curve, Cylinder, Plane, Revolution, Sphere, Surface, Torus};
use yack_math::kernel::{Frame, Point3, Transform, Vec3};

use super::{Cut, Stand, Sweep, leans, middle_param, seam_safe};
use yack_build::build::{Parts, param_of};
use yack_build::sweep::{Helix, screwed_side};

/// How `face` stands to a turn about the line through `origin` along the unit `axis`.
pub(super) fn stand(face: &PartFace, origin: Point3, axis: Vec3) -> Result<Stand> {
    let parallel = |v: Vec3| v.cross(axis).norm() <= yack_tol::ANGULAR;
    let on_axis = |p: Point3| (p - origin).cross(axis).norm() <= yack_tol::LINEAR;
    // The plane through `through` square to `axis × (through − origin)`, or along where that is
    // nothing (`through` on the axis).
    let contact = |through: Point3| -> Stand {
        match axis.cross(through - origin).unit() {
            Some(normal) if !on_axis(through) => {
                Stand::Contact(Cut::Plane { origin: through, normal }, None)
            }
            _ => Stand::Along,
        }
    };
    match &face.surface {
        Surface::Plane(p) => {
            let n = p.frame.z;
            if parallel(n) {
                return Ok(Stand::Along);
            }
            let normal = n
                .cross(axis)
                .unit()
                .ok_or_else(|| Error::Unsupported("by faces: a degenerate plane".into()))?;
            Ok(Stand::Contact(Cut::Plane { origin, normal }, None))
        }
        Surface::Sphere(s) => Ok(contact(s.frame.origin)),
        Surface::Cylinder(c) if parallel(c.frame.z) => Ok(contact(c.frame.origin)),
        Surface::Cone(c) if parallel(c.frame.z) && on_axis(c.frame.origin) => Ok(Stand::Along),
        Surface::Torus(t) if parallel(t.frame.z) && on_axis(t.frame.origin) => Ok(Stand::Along),
        other => Err(Error::Unsupported(format!(
            "by faces: a {} face a turn crosses is not built yet",
            other.kind()
        ))),
    }
}

/// The surface a piece of boundary sweeps as it turns by `angle` about the line through `origin`
/// along the unit `axis`, and whether the side faces against it; `curve` is already
/// seam-safe for the piece from `ends.0` to `ends.1`, `reversed` whether the leading face's loop
/// runs against it. A straight piece along the axis sweeps a cylinder, one square to it a plane;
/// anything else what [`whole_side`] makes of it. The surface's own seam (where its turn parameter
/// starts) goes opposite the side, so no side face meets it; on a whole turn (`angle` = `TAU`)
/// the side meets itself all the way round, its seam is the piece, and the surface is the
/// piece's revolution whatever the piece.
pub(super) fn side(
    curve: &Curve,
    ends: (Point3, Point3),
    reversed: bool,
    origin: Point3,
    axis: Vec3,
    angle: f64,
) -> Result<(Surface, bool)> {
    let (mid, d) = curve.d1(middle_param(curve, ends));
    let travel = if reversed { -d } else { d };
    // As for a slide: the face leads, so `travel × velocity` points away from its material.
    let velocity = axis.cross(mid - origin);
    let Some(out) = travel.cross(velocity).unit() else {
        return Err(Error::Unsupported("by faces: an edge that runs along its turn".into()));
    };
    let foot = |p: Point3| origin + axis * axis.dot(p - origin);
    let whole = angle >= std::f64::consts::TAU;
    // Half way round the turn, and then half a turn on: off the side.
    let away = Transform::rotation(origin, axis, 0.5 * angle + std::f64::consts::PI);
    let surface = match curve {
        _ if whole => whole_side(curve, ends, mid, origin, axis)?,
        Curve::Line(l) if l.dir.cross(axis).norm() <= yack_tol::ANGULAR => {
            let radial = mid - foot(mid);
            let x = away.vector(radial);
            Surface::Cylinder(Cylinder {
                frame: Frame::new(foot(mid), axis, x)?,
                radius: radial.norm(),
            })
        }
        Curve::Line(l) if l.dir.dot(axis).abs() <= yack_tol::ANGULAR => {
            Surface::Plane(Plane { frame: Frame::new(mid, out, l.dir)? })
        }
        // Anything else as a whole turn sweeps it (a torus or sphere for a circle in a plane
        // through the axis, a cone for a line in one), its seam turned away from the side.
        _ => whole_side(
            &curve.transformed(&away),
            (away.point(ends.0), away.point(ends.1)),
            away.point(mid),
            origin,
            axis,
        )?,
    };
    let uv = surface.project(mid);
    let natural = surface.normal(uv.x, uv.y);
    Ok((surface, natural.dot(out) < 0.0))
}

/// The surface a piece sweeps going all the way round, its seam (where its turn parameter starts)
/// on the piece: the quadric or torus of revolution it is where it is one, else its surface of
/// revolution. A straight piece along the axis sweeps a cylinder, one in a plane through the axis
/// a cone, one square to the axis a plane (whose side is an annulus or a disc: see
/// `super::annulus`); a circle in a plane through the axis a sphere (centred on the axis) or a
/// torus.
fn whole_side(
    curve: &Curve,
    ends: (Point3, Point3),
    mid: Point3,
    origin: Point3,
    axis: Vec3,
) -> Result<Surface> {
    let foot = |p: Point3| origin + axis * axis.dot(p - origin);
    let radial = |p: Point3| p - foot(p);
    let revolution =
        || Surface::Revolution(Box::new(Revolution { curve: curve.clone(), origin, axis }));
    Ok(match curve {
        Curve::Line(l) => {
            let in_meridian = l.dir.cross(axis).dot(mid - origin).abs() <= yack_tol::LINEAR;
            if l.dir.cross(axis).norm() <= yack_tol::ANGULAR {
                Surface::Cylinder(Cylinder {
                    frame: Frame::new(foot(mid), axis, radial(mid))?,
                    radius: radial(mid).norm(),
                })
            } else if l.dir.dot(axis).abs() <= yack_tol::ANGULAR {
                Surface::Plane(Plane { frame: Frame::new(mid, axis, radial(mid))? })
            } else if in_meridian {
                // From the end lower along the axis, so the generator runs up it.
                let (p1, p2) = if (ends.1 - ends.0).dot(axis) >= 0.0 {
                    (ends.0, ends.1)
                } else {
                    (ends.1, ends.0)
                };
                let rise = (p2 - p1).dot(axis);
                let spread = radial(p2).norm() - radial(p1).norm();
                let x = if radial(p1).norm() > yack_tol::LINEAR { radial(p1) } else { radial(p2) };
                Surface::Cone(Cone {
                    frame: Frame::new(foot(p1), axis, x)?,
                    radius: radial(p1).norm(),
                    half_angle: spread.atan2(rise),
                })
            } else {
                revolution()
            }
        }
        Curve::Circle(c) => {
            let n = c.frame.z;
            let centre = c.frame.origin;
            let in_meridian = n.dot(axis).abs() <= yack_tol::ANGULAR
                && n.dot(centre - origin).abs() <= yack_tol::LINEAR;
            let off = radial(centre).norm();
            if !in_meridian {
                revolution()
            } else if off <= yack_tol::LINEAR {
                Surface::Sphere(Sphere {
                    frame: Frame::new(centre, axis, radial(mid))?,
                    radius: c.radius,
                })
            } else if c.radius < off {
                Surface::Torus(Torus {
                    frame: Frame::new(foot(centre), axis, radial(centre))?,
                    major: off,
                    minor: c.radius,
                })
            } else {
                revolution()
            }
        }
        _ => revolution(),
    })
}

/// How `face` of `parts` stands to a screw about the line through `origin` along the unit `axis`
/// rising `rise` a radian, its velocity `axis × (p − origin) + rise·axis`. Contacts as for a turn,
/// shifted by the rise:
/// - a plane with normal `n` touches where `(p − origin)·(n × axis) + rise·(n·axis) = 0`, a plane
///   square to `n × axis`; a plane square to the axis leads (or trails) all over;
/// - a sphere about `c` on the plane through `c` square to `axis × (c − origin) + rise·axis`;
/// - a cylinder parallel to the axis as for a turn (its normal has no part along the axis);
/// - a cone about the axis leans one way all over, a torus about it touches on its equator plane.
///
/// Every other face declines. A screw is swept a quarter turn at a time (see [`super::swept`]).
pub(super) fn screw_stand(
    face: &PartFace,
    parts: &Parts,
    sweep: &Sweep,
    origin: Point3,
    axis: Vec3,
    rise: f64,
) -> Result<Stand> {
    let parallel = |v: Vec3| v.cross(axis).norm() <= yack_tol::ANGULAR;
    let on_axis = |p: Point3| (p - origin).cross(axis).norm() <= yack_tol::LINEAR;
    match &face.surface {
        Surface::Plane(p) => {
            let n = p.frame.z;
            let w = n.cross(axis);
            let Some(normal) = w.unit() else {
                return leans(face, parts, sweep);
            };
            let offset = -rise * n.dot(axis) / w.norm();
            Ok(Stand::Contact(Cut::Plane { origin: origin + normal * offset, normal }, None))
        }
        Surface::Sphere(s) => {
            let c = s.frame.origin;
            let normal = (axis.cross(c - origin) + axis * rise)
                .unit()
                .ok_or_else(|| Error::Unsupported("by faces: a still screw".into()))?;
            Ok(Stand::Contact(Cut::Plane { origin: c, normal }, None))
        }
        Surface::Cylinder(c) if parallel(c.frame.z) => {
            match axis.cross(c.frame.origin - origin).unit() {
                Some(normal) if !on_axis(c.frame.origin) => {
                    Ok(Stand::Contact(Cut::Plane { origin: c.frame.origin, normal }, None))
                }
                _ => Ok(Stand::Along),
            }
        }
        Surface::Cone(c) if parallel(c.frame.z) && on_axis(c.frame.origin) => {
            leans(face, parts, sweep)
        }
        Surface::Torus(t) if parallel(t.frame.z) && on_axis(t.frame.origin) => {
            Ok(Stand::Contact(Cut::Plane { origin: t.frame.origin, normal: axis }, None))
        }
        other => Err(Error::Unsupported(format!(
            "by faces: a {} face a screw crosses is not built yet",
            other.kind()
        ))),
    }
}

/// The surface a piece of boundary sweeps through `angle` of `helix`, and whether the side faces
/// against it: yack-build's screwed side of the piece from `ends.0` to `ends.1` (the exact
/// cylinder for a piece along the axis, else a rational surface within its screw fit).
pub(super) fn screw_side(
    curve: &Curve,
    ends: (Point3, Point3),
    reversed: bool,
    helix: &Helix,
    angle: f64,
) -> Result<(Surface, bool)> {
    let curve = &seam_safe(curve, ends)?;
    let t0 = param_of(curve, ends.0);
    let t1 = 2.0 * middle_param(curve, ends) - t0;
    let (mid, d) = curve.d1(middle_param(curve, ends));
    let travel = if reversed { -d } else { d };
    let velocity = helix.axis.cross(mid - helix.origin) + helix.axis * helix.rise;
    let Some(out) = travel.cross(velocity).unit() else {
        return Err(Error::Unsupported("by faces: an edge that runs along its screw".into()));
    };
    // A circle about the axis itself screws along its own cylinder: that cylinder, exactly (a
    // fitted copy of it would lie on the solid's own face, which the union cannot sort out).
    let coaxial = match curve {
        Curve::Circle(c) => {
            c.frame.z.cross(helix.axis).norm() <= yack_tol::ANGULAR
                && (c.frame.origin - helix.origin).cross(helix.axis).norm() <= yack_tol::LINEAR
        }
        _ => false,
    };
    let surface = match curve {
        Curve::Circle(c) if coaxial => {
            let x = mid - c.frame.origin;
            Surface::Cylinder(Cylinder {
                frame: Frame::new(c.frame.origin, helix.axis, x)?,
                radius: c.radius,
            })
        }
        _ => screwed_side(helix, curve, (t0, t1), angle)?,
    };
    let uv = surface.project(mid);
    let natural = surface.normal(uv.x, uv.y);
    Ok((surface, natural.dot(out) < 0.0))
}
