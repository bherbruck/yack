// yack: brep-bench case edit/frustum-rim-bevel-deleted
//! A cone frustum's beveled top rim deleted: the top and the cone meet again.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let part = cone(&Frame::WORLD, 6.0, 3.5, 5.0)?.union(&cone(&Frame::at([0.0, 0.0, 5.0]), 3.5, 2.0, 1.0)?)?;
    part.delete_faces(&[part.face_at([0.0, 2.75, 5.5])?])
}
