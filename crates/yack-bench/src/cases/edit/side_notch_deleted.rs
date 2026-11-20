// yack: brep-bench case edit/side-notch-deleted
//! A half-round notch in a block's side deleted: the side closes over it.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let b = block(&Frame::WORLD, [20.0, 10.0, 10.0])?.difference(&cylinder(&Frame::at([10.0, 0.0, -1.0]), 3.0, 12.0)?)?;
    b.delete_faces(&[b.face_at([10.0, 3.0, 5.0])?])
}
