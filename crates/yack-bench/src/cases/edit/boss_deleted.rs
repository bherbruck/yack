// yack: brep-bench case edit/boss-deleted
//! A plate's boss deleted: the plate's top closes over where it stood.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let part = cylinder(&Frame::at([0.0, 0.0, -4.0]), 8.0, 4.0)?.union(&cylinder(&Frame::WORLD, 3.0, 8.0)?)?;
    part.delete_faces(&[part.face_at([0.0, 3.0, 4.0])?, part.face_at([0.0, 0.0, 8.0])?])
}
