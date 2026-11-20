// yack: glue for yack's edits: offsets, face moves, drafts and transforms over springrep's `edit`.
//! The edits `yack::Solid` exposes, as springrep calls with the protocol's argument shapes.

use std::collections::HashMap;

use crate::{
    Result,
    edit,
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
    edit::offset(b, distance, "offset")
}

/// Offsets each of `faces` by the same `distance`.
pub fn offset_faces(b: &Brep, faces: &[FaceId], distance: f64) -> Result<Brep> {
    let by: HashMap<FaceId, f64> = faces.iter().map(|&f| (f, distance)).collect();
    edit::offset_faces(b, &by)
}

/// Removes `faces`, healing from their neighbours.
pub fn delete_faces(b: &Brep, faces: &[FaceId]) -> Result<Brep> {
    edit::delete_faces(b, faces)
}

/// Moves `faces` by the rigid motion with rows `m`.
pub fn move_faces(b: &Brep, faces: &[FaceId], m: [[f64; 4]; 3]) -> Result<Brep> {
    let (r, t) = split(m);
    edit::move_faces_by(b, faces, &Transform::rigid(r, t)?)
}

/// Thickens `faces` by `thickness`.
pub fn thicken(b: &Brep, faces: &[FaceId], thickness: f64) -> Result<Brep> {
    edit::thicken(b, faces, thickness)
}

/// Hollows to a wall of `thickness` inside, removing `open`.
pub fn shell(b: &Brep, open: &[FaceId], thickness: f64) -> Result<Brep> {
    edit::shell(b, open, thickness, edit::Wall::Inward, "shell")
}

/// Drafts `faces` by `angle` radians about the neutral plane through `origin` normal to `pull`.
pub fn draft(b: &Brep, faces: &[FaceId], origin: [f64; 3], pull: [f64; 3], angle: f64) -> Result<Brep> {
    let z = v3(pull);
    let hint = if z.x.abs() < 0.9 { Vec3::new(1.0, 0.0, 0.0) } else { Vec3::new(0.0, 1.0, 0.0) };
    let neutral = Frame::new(v3(origin), z, hint)?;
    edit::draft(b, faces, &neutral, angle)
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
    edit::affine(b, &edit::Affine { m, t })
}
