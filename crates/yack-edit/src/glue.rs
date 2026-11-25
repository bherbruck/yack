// yack: glue for yack's edits: offsets, face moves, drafts and transforms over this crate's edits (moved from springrep's facade/edit.rs, M3 W10).
//! The edits `yack::Solid` exposes, as springrep calls with the protocol's argument shapes.

#[allow(clippy::disallowed_types)] use std::collections::HashMap; // D26: BTree* in hardening, proven by digests

use crate::{
    Result,
    math::{Frame, Transform, Vec3},
    topo::{Brep, FaceId},
};

fn v3(a: [f64; 3]) -> Vec3 {
    Vec3::new(a[0], a[1], a[2])
}

/// The 3×3 part and the translation of a 3×4 row matrix.
fn split(m: [[f64; 4]; 3]) -> ([[f64; 3]; 3], Vec3) {
    (m.map(|r| [r[0], r[1], r[2]]), Vec3::new(m[0][3], m[1][3], m[2][3]))
}

/// Offsets every face by `distance` (positive grows).
pub fn offset(b: &Brep, distance: f64) -> Result<Brep> {
    crate::offset(b, distance, "offset")
}

/// Offsets each of `faces` by the same `distance`.
#[allow(clippy::disallowed_types)] fn offset_faces(b: &Brep, faces: &[FaceId], distance: f64) -> Result<Brep> { // D26: BTree* in hardening, proven by digests
    let by: HashMap<FaceId, f64> = faces.iter().map(|&f| (f, distance)).collect();
    crate::offset_faces(b, &by)
}

/// Removes `faces`, healing from their neighbours.
fn delete_faces(b: &Brep, faces: &[FaceId]) -> Result<Brep> {
    crate::delete_faces(b, faces)
}

/// Moves `faces` by the rigid motion with rows `m`.
fn move_faces(b: &Brep, faces: &[FaceId], m: [[f64; 4]; 3]) -> Result<Brep> {
    let (r, t) = split(m);
    crate::move_faces_by(b, faces, &Transform::rigid(r, t)?)
}

/// Thickens `faces` by `thickness`.
fn thicken(b: &Brep, faces: &[FaceId], thickness: f64) -> Result<Brep> {
    crate::thicken(b, faces, thickness)
}

/// Hollows to a wall of `thickness` inside, removing `open`.
pub fn shell(b: &Brep, open: &[FaceId], thickness: f64) -> Result<Brep> {
    crate::shell(b, open, thickness, crate::Wall::Inward, "shell")
}

/// Wraps the solid in a wall of `thickness` outside it, removing `open`: the faces become the wall's inside.
pub fn shell_outward(b: &Brep, open: &[FaceId], thickness: f64) -> Result<Brep> {
    crate::shell(b, open, thickness, crate::Wall::Outward, "shell")
}

/// Drafts `faces` by `angle` radians about the neutral plane through `origin` normal to `pull`.
fn draft(b: &Brep, faces: &[FaceId], origin: [f64; 3], pull: [f64; 3], angle: f64) -> Result<Brep> {
    let z = v3(pull);
    let hint = if z.x.abs() < 0.9 { Vec3::new(1.0, 0.0, 0.0) } else { Vec3::new(0.0, 1.0, 0.0) };
    let neutral = Frame::new(v3(origin), z, hint)?;
    crate::draft(b, faces, &neutral, angle)
}

/// The image under the similarity with rows `m` (a stretch or shear is refused).
pub fn transform(b: &Brep, m: [[f64; 4]; 3]) -> Result<Brep> {
    let (r, t) = split(m);
    let mut out = b.clone();
    out.transform_similar(r, t)?;
    Ok(out)
}

/// The image under the invertible affine map with rows `m`.
pub fn affine(b: &Brep, m: [[f64; 4]; 3]) -> Result<Brep> {
    let (m, t) = split(m);
    crate::affine(b, &crate::Affine { m, t })
}

/// `b` with every face that has no name named `<feature>.<part>`: springrep's face edits keep the
/// names of the faces they carry over and leave the faces they make unnamed.
fn name_new(mut b: Brep, feature: &str, part: &str) -> Brep {
    let fresh: Vec<FaceId> = b.face_ids().filter(|&f| b.face(f).names.is_empty()).collect();
    for f in fresh {
        b.set_face_names(f, std::iter::once(crate::topo::FaceName::new(feature, part)).collect());
    }
    b
}

/// [`offset_faces`], naming the faces it makes `<feature>.side`.
pub fn offset_faces_named(b: &Brep, faces: &[FaceId], distance: f64, feature: &str) -> Result<Brep> {
    offset_faces(b, faces, distance).map(|r| name_new(r, feature, "side"))
}

/// [`delete_faces`], naming the faces it makes `<feature>.fill`.
pub fn delete_faces_named(b: &Brep, faces: &[FaceId], feature: &str) -> Result<Brep> {
    delete_faces(b, faces).map(|r| name_new(r, feature, "fill"))
}

/// [`move_faces`], naming the faces it makes `<feature>.side`.
pub fn move_faces_named(b: &Brep, faces: &[FaceId], m: [[f64; 4]; 3], feature: &str) -> Result<Brep> {
    move_faces(b, faces, m).map(|r| name_new(r, feature, "side"))
}

/// [`thicken`], naming the faces it makes `<feature>.side`.
pub fn thicken_named(b: &Brep, faces: &[FaceId], thickness: f64, feature: &str) -> Result<Brep> {
    thicken(b, faces, thickness).map(|r| name_new(r, feature, "side"))
}

/// [`draft`], naming the faces it makes `<feature>.side`.
pub fn draft_named(
    b: &Brep,
    faces: &[FaceId],
    origin: [f64; 3],
    pull: [f64; 3],
    angle: f64,
    feature: &str,
) -> Result<Brep> {
    draft(b, faces, origin, pull, angle).map(|r| name_new(r, feature, "side"))
}
