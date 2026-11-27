// yack: the cavity of a cone frustum's inward shell that closes to a point before it reaches the far face
//! A frustum shelled inward has a cone for a cavity. Where that cone comes to its apex before the
//! frustum's far face, the far face never bounds the cavity, and moving it past the apex has no
//! meaning: the cavity is the cavity of the frustum's whole cone, its apex completed.

use std::collections::BTreeMap;

use crate::{
    Error, Result,
    geom::Surface,
    math::{Frame, Point3},
    tol,
    topo::{Brep, FaceId, local::face_normal_at},
};

/// The cavity of `s` shelled inward, and the face of `s` each of its faces offsets, by `t`, open through `open`, when `s` is a frustum (a cone side
/// between two planes square to its axis) whose cavity closes to its apex before the plane nearer
/// that apex; `None` for any other solid or a cavity that does not close early.
pub(crate) fn closed_cavity(s: &Brep, open: &[FaceId], t: f64) -> Result<Option<(Brep, BTreeMap<FaceId, FaceId>)>> {
    let faces: Vec<FaceId> = s.face_ids().collect();
    let &[_, _, _] = faces.as_slice() else { return Ok(None) };
    let Some(&side) = faces.iter().find(|&&f| matches!(s.face(f).surface, Surface::Cone(_))) else {
        return Ok(None);
    };
    let Surface::Cone(c) = &s.face(side).surface else { return Ok(None) };
    let planes: Vec<(FaceId, Frame)> = faces
        .iter()
        .filter_map(|&f| match &s.face(f).surface {
            Surface::Plane(p) => Some((f, p.frame)),
            _ => None,
        })
        .collect();
    let &[(f0, p0), (f1, p1)] = planes.as_slice() else { return Ok(None) };
    let (sa, ca) = c.half_angle.sin_cos();
    let z = c.frame.z;
    let axial = |p: Point3| (p - c.frame.origin).dot(z);
    let square = |p: &Frame| p.z.cross(z).norm() <= 1e-6; // tol: a plane this far from square is not a frustum's
    if sa.abs() <= 1e-9 || !square(&p0) || !square(&p1) { // tol: a half angle this small is a cylinder
        return Ok(None);
    }
    let apex = -c.radius / sa * ca;
    let held = [(f0, p0, axial(p0.origin)), (f1, p1, axial(p1.origin))];
    let [near, far] = if (apex - held[0].2).abs() <= (apex - held[1].2).abs() { [held[0], held[1]] } else { [held[1], held[0]] };
    let toward = (apex - far.2).signum();
    // Where each plane goes, an open one out and a closed one in, and the cavity's apex: the cone
    // moved in along its normal by `dn` has its apex `dn / sin a` along the axis from the old.
    let reaches = |(f, frame, _): (FaceId, Frame, f64)| -> Option<f64> {
        let out = face_normal_at(s, f, frame.origin)?;
        let by = if open.contains(&f) { t } else { -t };
        Some(axial(frame.origin + out * by))
    };
    let dn = if s.face(side).reversed { t } else { -t };
    let cavity_apex = apex - dn / sa;
    let (Some(near_at), Some(far_at)) = (reaches(near), reaches(far)) else { return Ok(None) };
    if (near_at - cavity_apex) * toward <= tol::LINEAR || (cavity_apex - far_at) * toward <= tol::LINEAR {
        return Ok(None);
    }
    // The cone completed to its apex, on the far plane: its cavity is the one wanted.
    let base = (c.radius + far.2 / ca * sa).max(0.0);
    if base <= tol::LINEAR {
        return Ok(None);
    }
    let origin = c.frame.origin + z * far.2;
    let frame = Frame::new(origin, z * toward, c.frame.x)?;
    let whole = yack_build::primitive::cone(&frame, base, 0.0, (apex - far.2).abs(), "cone")?;
    let faces: Vec<FaceId> = whole.face_ids().collect();
    let (Some(&cone_side), Some(&floor)) = (
        faces.iter().find(|&&f| matches!(whole.face(f).surface, Surface::Cone(_))),
        faces.iter().find(|&&f| matches!(whole.face(f).surface, Surface::Plane(_))),
    ) else {
        return Err(Error::Unsupported("the whole cone of a frustum".into()));
    };
    let by: BTreeMap<FaceId, f64> = [(cone_side, -t), (floor, if open.contains(&far.0) { t } else { -t })].into_iter().collect();
    let cavity = crate::offset_faces(&whole, &by)?;
    // Each cavity face offsets a face of `s`: the cone's side the frustum's side, its floor the far plane.
    let mut from = BTreeMap::new();
    if crate::same_topology(&whole, &cavity) {
        from.insert(cone_side, side);
        from.insert(floor, far.0);
    }
    Ok(Some((cavity, from)))
}
