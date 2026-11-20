// yack: edits on `Solid` (signed distances: positive grows), routed to springrep's `edit` through
// its facade glue (M1.4). `swept` stays a stub: springrep has no swept-solid operation.
use yack_springrep::facade::edit as sr;

use crate::firewall;
use crate::solid::engine_ids;
use crate::{Affine, Angle, Error, FaceId, Motion, Point3, Result, Rigid, Similarity, Solid, Vec3, engine};

impl Solid {
    /// The engine faces for `ids`, each checked to belong to this solid.
    fn sr_faces(&self, op: &'static str, arg: &'static str, ids: &[FaceId]) -> Result<Vec<engine_ids::Face>> {
        ids.iter().map(|&f| self.sr_face(op, arg, f)).collect()
    }

    /// Offsets every face by `distance` (positive grows).
    pub fn offset(&self, distance: f64) -> Result<Solid> {
        let op = "offset";
        let Some(b) = self.brep() else { return Ok(Solid::empty()) };
        firewall::run(op, || engine::solid(op, sr::offset(b, distance)))
    }
    /// Offsets `faces` by `distance` (positive grows), extending their neighbours.
    pub fn offset_faces(&self, faces: &[FaceId], distance: f64) -> Result<Solid> {
        let op = "offset_faces";
        let f = self.sr_faces(op, "faces", faces)?;
        firewall::run(op, || engine::solid(op, sr::offset_faces(self.need(op)?, &f, distance)))
    }
    /// Removes `faces` and heals the gap from the neighbours.
    pub fn delete_faces(&self, faces: &[FaceId]) -> Result<Solid> {
        let op = "delete_faces";
        let f = self.sr_faces(op, "faces", faces)?;
        firewall::run(op, || engine::solid(op, sr::delete_faces(self.need(op)?, &f)))
    }
    /// Moves `faces` by the rigid motion `by`, extending their neighbours.
    pub fn move_faces(&self, faces: &[FaceId], by: &Rigid) -> Result<Solid> {
        let op = "move_faces";
        let f = self.sr_faces(op, "faces", faces)?;
        firewall::run(op, || engine::solid(op, sr::move_faces(self.need(op)?, &f, by.rows())))
    }
    /// Thickens `faces` by `thickness` (positive grows outward) into a new solid.
    pub fn thicken(&self, faces: &[FaceId], thickness: f64) -> Result<Solid> {
        let op = "thicken";
        let f = self.sr_faces(op, "faces", faces)?;
        firewall::run(op, || engine::solid(op, sr::thicken(self.need(op)?, &f, thickness)))
    }
    /// Hollows the solid to a wall of `thickness`, removing the `open` faces.
    pub fn shell(&self, open: &[FaceId], thickness: f64) -> Result<Solid> {
        let op = "shell";
        let f = self.sr_faces(op, "open", open)?;
        firewall::run(op, || engine::solid(op, sr::shell(self.need(op)?, &f, thickness)))
    }
    /// Hollows the solid by a wall of `thickness` grown outside it, removing the `open` faces: the
    /// solid's faces become the wall's inside.
    pub fn shell_outward(&self, open: &[FaceId], thickness: f64) -> Result<Solid> {
        let op = "shell_outward";
        let f = self.sr_faces(op, "open", open)?;
        firewall::run(op, || engine::solid(op, sr::shell_outward(self.need(op)?, &f, thickness)))
    }
    /// Tilts `faces` by `angle` about the neutral plane through `origin` normal to `pull`.
    pub fn draft(&self, faces: &[FaceId], origin: impl Into<Point3>, pull: impl Into<Vec3>, angle: Angle) -> Result<Solid> {
        let op = "draft";
        let (origin, pull) = (origin.into(), pull.into());
        if !origin.is_finite() {
            return Err(Error::invalid(op, "origin", "not finite"));
        }
        if !pull.is_finite() {
            return Err(Error::invalid(op, "pull", "not finite"));
        }
        let f = self.sr_faces(op, "faces", faces)?;
        firewall::run(op, || {
            engine::solid(op, sr::draft(self.need(op)?, &f, origin.to_array(), pull.to_array(), angle.radians()))
        })
    }
    /// Every point the solid passes through during `motion`.
    pub fn swept(&self, motion: &Motion) -> Result<Solid> {
        let _ = motion;
        Err(Error::unsupported("swept"))
    }

    /// The image under a similarity (rigid, uniform scale, mirror).
    pub fn transform(&self, m: &Similarity) -> Result<Solid> {
        let op = "transform";
        let Some(b) = self.brep() else { return Ok(Solid::empty()) };
        firewall::run(op, || engine::solid(op, sr::transform(b, m.rows())))
    }

    /// The image under an invertible affine map (protocol `affine`).
    pub fn transform_affine(&self, m: &Affine) -> Result<Solid> {
        let op = "transform_affine";
        let Some(b) = self.brep() else { return Ok(Solid::empty()) };
        firewall::run(op, || engine::solid(op, sr::affine(b, m.rows())))
    }
}
