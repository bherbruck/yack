// yack: a draft: faces tilted about where they meet a neutral plane (from lib.rs, hardening T1).
// Adapted from springrep@a03e8a1 src/edit/tweak.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from edit/tweak.rs to tweak/mod.rs (M2.4, T6) and to yack-tweak (M3 W7); split out of
// lib.rs by what it does (hardening T1); errors for its two broken invariants, not panics (hardening T2).
//! A draft: each face tilted about the line where it meets the neutral plane, a cylinder along the pull
//! made the cone through its circle there, and followed through where faces close on the way.

use std::collections::BTreeMap;
use crate::{Error, Result, geom::Cone, geom::Plane, geom::Surface, math::Frame, topo::Brep, topo::FaceId, unsupported, plane_of, followed, Moved, through, tol, planes::planes_or_faces, surfaces::move_faces};

/// `b` with each of `faces` tilted by `angle` about the line where it meets the `neutral` plane, its
/// outward normal turned towards the plane's normal: positive angles narrow the part along it.
pub fn draft(b: &Brep, faces: &[FaceId], neutral: &Frame, angle: f64) -> Result<Moved> {
    let k = neutral.z;
    let mut moved = BTreeMap::new();
    let mut cones = BTreeMap::new();
    for &g in faces {
        let face = b.face(g);
        // A cylinder along the pull becomes the cone through its circle on the neutral plane:
        // narrowing along the pull where the material lies inside it, widening a bore.
        if let Surface::Cylinder(c) = &face.surface
            && c.frame.z.cross(k).norm() <= tol::ANGULAR
        {
            let level = (neutral.origin - c.frame.origin).dot(k) / c.frame.z.dot(k);
            let frame = Frame::new(c.frame.origin + c.frame.z * level, k, c.frame.x)?;
            let half_angle = if face.reversed { angle } else { -angle };
            cones.insert(
                g,
                Surface::Cone(Cone {
                    frame,
                    radius: c.radius,
                    half_angle,
                }),
            );
            continue;
        }
        let Some(fr) = plane_of(&face.surface) else {
            return unsupported(&format!("drafting a {} face", face.surface.kind()));
        };
        let n = if face.reversed { -fr.z } else { fr.z };
        let (Some(level), Some(up)) = ((n - k * n.dot(k)).unit(), (k - n * n.dot(k)).unit()) else {
            return Err(Error::InvalidInput(
                "a face parallel to the neutral plane has no draft".into(),
            ));
        };
        let hinge = fr.origin + up * ((neutral.origin - fr.origin).dot(k) / up.dot(k));
        let tilted = level * angle.cos() + k * angle.sin();
        let z = if face.reversed { -tilted } else { tilted };
        moved.insert(g, Frame::new(hinge, z, fr.x)?);
    }
    // Where a face closes on the way, walls leaning until they meet, the draft is followed there.
    let mut paths: BTreeMap<FaceId, through::Path> = BTreeMap::new();
    for (&g, &to) in &moved {
        let Some(from) = plane_of(&b.face(g).surface) else {
            return Err(Error::InvalidResult(vec!["a drafted plane is no longer a plane".into()]));
        };
        paths.insert(
            g,
            Box::new(move |s| {
                Ok(Surface::Plane(Plane {
                    frame: through::pencil(&from, &to, s)?,
                }))
            }),
        );
    }
    for (&g, cone) in &cones {
        let Surface::Cone(cone) = cone.clone() else {
            return Err(Error::InvalidResult(vec!["a drafted cylinder is no longer a cone".into()]));
        };
        paths.insert(g, Box::new(move |s| Ok(through::opening(&cone, s))));
    }
    let direct = if cones.is_empty() {
        planes_or_faces(b, &moved)
    } else {
        let mut surfaces = cones.clone();
        for (&g, &frame) in &moved {
            surfaces.insert(g, Surface::Plane(Plane { frame }));
        }
        move_faces(b, &surfaces)
    };
    followed(direct, || through::through(b, &paths))
}
