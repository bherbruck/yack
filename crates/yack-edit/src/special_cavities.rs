// yack: the cavities of an inward shell that the face-by-face offset cannot move: where a surface closes or is tangent to the move
//! Three shapes of turned solid whose cavity is known in closed form, and which the offset of their
//! faces one by one cannot reach, because two of the moved surfaces touch or lose each other:
//!
//! - a ball cut by a plane and opened there (a cap): the opening's plane moved out by the wall is
//!   tangent to or past the ball shrunk by the wall, so the cavity is that ball whole;
//! - a cylinder ending in a hemispherical dome, opened at the dome: the dome's sphere and the cylinder
//!   meet tangentially, so moved they meet nowhere a corner can follow; the cavity is the cylinder
//!   shrunk by the wall, run out through the dome;
//! - a cone whose apex is rounded by a ball no larger than the wall: the ball closes, and the cavity is
//!   the cavity of the sharp cone.
//!
//! Each declines (`None`) a solid that is not exactly that shape.

use std::collections::BTreeMap;

use crate::{
    Result,
    geom::Surface,
    math::{Frame, Point3, Vec3},
    tol,
    topo::{Brep, FaceId, local::face_normal_at},
};

type Cavity = (Brep, BTreeMap<FaceId, FaceId>);

/// The cavity of `s` shelled inward by `t` and opened through `open`, when `s` is one of the three
/// shapes above; the face of `s` each cavity face offsets goes with it.
pub(crate) fn cavity(s: &Brep, open: &[FaceId], t: f64) -> Result<Option<Cavity>> {
    for build in [ball_cap, capsule, ball_tipped_cone] {
        if let Some(found) = build(s, open, t)? {
            return Ok(Some(found));
        }
    }
    Ok(None)
}

/// The faces of `s` of each kind: planes, cylinders, cones, spheres.
fn kinds(s: &Brep) -> (Vec<FaceId>, Vec<FaceId>, Vec<FaceId>, Vec<FaceId>) {
    let pick = |want: fn(&Surface) -> bool| s.face_ids().filter(|&f| want(&s.face(f).surface)).collect::<Vec<_>>();
    (
        pick(|x| matches!(x, Surface::Plane(_))),
        pick(|x| matches!(x, Surface::Cylinder(_))),
        pick(|x| matches!(x, Surface::Cone(_))),
        pick(|x| matches!(x, Surface::Sphere(_))),
    )
}

fn plane_frame(s: &Brep, f: FaceId) -> Option<(Point3, Vec3)> {
    match &s.face(f).surface {
        Surface::Plane(p) => Some((p.frame.origin, if s.face(f).reversed { -p.frame.z } else { p.frame.z })),
        _ => None,
    }
}

/// A ball cut by a plane (two faces), opened at the plane.
fn ball_cap(s: &Brep, open: &[FaceId], t: f64) -> Result<Option<Cavity>> {
    let (planes, cylinders, cones, spheres) = kinds(s);
    let ([plane], [ball], true) = (&planes[..], &spheres[..], cylinders.is_empty() && cones.is_empty() && s.face_ids().count() == 2) else {
        return Ok(None);
    };
    let (Surface::Sphere(sp), Some((o, n)), [opened]) = (&s.face(*ball).surface, plane_frame(s, *plane), open) else {
        return Ok(None);
    };
    let inner = sp.radius - t;
    // The plane moved out by `t`, as far from the centre as the shrunk ball reaches or farther.
    let moved = (o - sp.frame.origin).dot(n) + t;
    if opened != plane || s.face(*ball).reversed || inner <= tol::LINEAR || moved < inner - tol::LINEAR {
        return Ok(None);
    }
    let whole = yack_build::primitive::sphere(&Frame::new(sp.frame.origin, sp.frame.z, sp.frame.x)?, inner, "ball")?;
    let from = whole.face_ids().map(|f| (f, *ball)).collect();
    Ok(Some((whole, from)))
}

/// A cylinder with a plane at one end and a hemisphere of its own radius at the other, opened at the dome.
fn capsule(s: &Brep, open: &[FaceId], t: f64) -> Result<Option<Cavity>> {
    let (planes, cylinders, cones, spheres) = kinds(s);
    let ([floor], [side], [dome], true) = (&planes[..], &cylinders[..], &spheres[..], cones.is_empty() && s.face_ids().count() == 3) else {
        return Ok(None);
    };
    let (Surface::Cylinder(c), Surface::Sphere(sp), Some((o, n)), [opened]) = (&s.face(*side).surface, &s.face(*dome).surface, plane_frame(s, *floor), open) else {
        return Ok(None);
    };
    let axis = c.frame.z;
    let off = sp.frame.origin - c.frame.origin;
    let on_axis = (off - axis * off.dot(axis)).norm() <= tol::LINEAR;
    let tangent = (sp.radius - c.radius).abs() <= tol::LINEAR;
    let square = n.cross(axis).norm() <= 1e-9; // tol: a cosine
    let inner = c.radius - t;
    if opened != dome || !on_axis || !tangent || !square || inner <= tol::LINEAR || s.face(*side).reversed || s.face(*dome).reversed {
        return Ok(None);
    }
    // The cavity runs from the floor moved in by `t` toward the dome, past the dome's far pole.
    let toward = if (sp.frame.origin - o).dot(axis) >= 0.0 { axis } else { -axis };
    let start = o - n * t;
    let length = (sp.frame.origin - start).dot(toward) + sp.radius + t + 1.0;
    let rod = yack_build::primitive::cylinder(&Frame::new(start, toward, c.frame.x)?, inner, length, "rod")?;
    let mut from = BTreeMap::new();
    for f in rod.face_ids() {
        match &rod.face(f).surface {
            Surface::Cylinder(_) => {
                from.insert(f, *side);
            }
            Surface::Plane(_) if face_normal_at(&rod, f, start).is_some_and(|m| m.dot(toward) < 0.0) => {
                from.insert(f, *floor);
            }
            _ => {}
        }
    }
    Ok(Some((rod, from)))
}

/// A cone on a plane base whose apex is rounded by a ball tangent to it, the ball no larger than the wall.
fn ball_tipped_cone(s: &Brep, open: &[FaceId], t: f64) -> Result<Option<Cavity>> {
    let (planes, cylinders, cones, spheres) = kinds(s);
    let ([base], [side], [tip], true) = (&planes[..], &cones[..], &spheres[..], cylinders.is_empty() && s.face_ids().count() == 3) else {
        return Ok(None);
    };
    let (Surface::Cone(c), Surface::Sphere(sp), Some((o, n))) = (&s.face(*side).surface, &s.face(*tip).surface, plane_frame(s, *base)) else {
        return Ok(None);
    };
    let (sa, ca) = c.half_angle.sin_cos();
    let axis = c.frame.z;
    let valid_open = open.is_empty() || open == [*base];
    if !valid_open || sa.abs() <= 1e-9 || n.cross(axis).norm() > 1e-9 || sp.radius > t + tol::LINEAR || s.face(*side).reversed || s.face(*tip).reversed { // tol: a sine; a cosine
        return Ok(None);
    }
    let apex = c.frame.origin + axis * (-c.radius * ca / sa);
    let off = sp.frame.origin - apex;
    // The ball is centred on the axis, `r / sin a` from the apex: tangent to the cone.
    if (off - axis * off.dot(axis)).norm() > tol::LINEAR || (off.norm() - sp.radius / sa.abs()).abs() > 10.0 * tol::LINEAR {
        return Ok(None);
    }
    // The sharp cone: the base circle's radius where the base plane cuts the cone, up to the apex.
    let toward = if (apex - o).dot(axis) >= 0.0 { axis } else { -axis };
    let height = (apex - o).dot(toward);
    let radius = height * (sa / ca).abs();
    if height <= tol::LINEAR || radius <= tol::LINEAR {
        return Ok(None);
    }
    let sharp = yack_build::primitive::cone(&Frame::new(o, toward, c.frame.x)?, radius, 0.0, height, "cone")?;
    let (mut shell_side, mut shell_base) = (None, None);
    for f in sharp.face_ids() {
        match &sharp.face(f).surface {
            Surface::Cone(_) => shell_side = Some(f),
            Surface::Plane(_) => shell_base = Some(f),
            _ => {}
        }
    }
    let (Some(cone_face), Some(floor)) = (shell_side, shell_base) else {
        return Ok(None);
    };
    let by: BTreeMap<FaceId, f64> = [(cone_face, -t), (floor, if open.is_empty() { -t } else { t })].into_iter().collect();
    let cavity = crate::offset_faces(&sharp, &by)?;
    let mut from = BTreeMap::new();
    if crate::same_topology(&sharp, &cavity) {
        from.insert(cone_face, *side);
        from.insert(floor, *base);
    }
    Ok(Some((cavity, from)))
}
