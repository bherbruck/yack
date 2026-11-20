// yack: edits on `Solid` (signed distances: positive grows). M1.4 wires the face edits; the maps
// are wired here.
use crate::engine;
use crate::{Affine, Angle, Error, FaceId, Motion, Point3, Result, Rigid, Similarity, Solid, Vec3};

fn unsupported(op: &'static str) -> Result<Solid> {
    Err(Error::unsupported(op))
}

impl Solid {
    /// Offsets every face by `distance` (positive grows).
    pub fn offset(&self, distance: f64) -> Result<Solid> {
        let _ = distance;
        unsupported("offset")
    }
    /// Offsets `faces` by `distance` (positive grows), extending their neighbours.
    pub fn offset_faces(&self, faces: &[FaceId], distance: f64) -> Result<Solid> {
        let _ = (faces, distance);
        unsupported("offset_faces")
    }
    /// Removes `faces` and heals the gap from the neighbours.
    pub fn delete_faces(&self, faces: &[FaceId]) -> Result<Solid> {
        let _ = faces;
        unsupported("delete_faces")
    }
    /// Moves `faces` by the rigid motion `by`, extending their neighbours.
    pub fn move_faces(&self, faces: &[FaceId], by: &Rigid) -> Result<Solid> {
        let _ = (faces, by);
        unsupported("move_faces")
    }
    /// Thickens `faces` by `thickness` (positive grows outward) into a new solid.
    pub fn thicken(&self, faces: &[FaceId], thickness: f64) -> Result<Solid> {
        let _ = (faces, thickness);
        unsupported("thicken")
    }
    /// Hollows the solid to a wall of `thickness`, removing the `open` faces.
    pub fn shell(&self, open: &[FaceId], thickness: f64) -> Result<Solid> {
        let _ = (open, thickness);
        unsupported("shell")
    }
    /// Tilts `faces` by `angle` about the neutral plane through `origin` normal to `pull`.
    pub fn draft(&self, faces: &[FaceId], origin: impl Into<Point3>, pull: impl Into<Vec3>, angle: Angle) -> Result<Solid> {
        let _ = (faces, origin.into(), pull.into(), angle);
        unsupported("draft")
    }
    /// Every point the solid passes through during `motion`.
    pub fn swept(&self, motion: &Motion) -> Result<Solid> {
        let _ = motion;
        unsupported("swept")
    }

    /// The image under a similarity (rigid, uniform scale, mirror).
    pub fn transform(&self, m: &Similarity) -> Result<Solid> {
        let Some(b) = self.brep() else { return Ok(Solid::empty()) };
        let (r, t) = engine::split_rows(m.rows());
        let mut out = b.clone();
        out.transform_similar(r, t).map_err(|e| engine::error("transform", e))?;
        Ok(Solid::from_springrep(out))
    }

    /// The image under an invertible affine map (protocol `affine`).
    pub fn transform_affine(&self, m: &Affine) -> Result<Solid> {
        let Some(b) = self.brep() else { return Ok(Solid::empty()) };
        let (m, t) = engine::split_rows(m.rows());
        engine::solid("transform_affine", yack_springrep::edit::affine(b, &yack_springrep::edit::Affine { m, t }))
    }
}
