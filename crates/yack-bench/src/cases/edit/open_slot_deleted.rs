//! An open slot in a block's side deleted: the side closes over it.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let slot = block(&Frame::at([8.0, -1.0, -1.0]), [4.0, 6.0, 12.0])?;
    let slot = slot.union(&cylinder(&Frame::at([10.0, 5.0, -1.0]), 2.0, 12.0)?)?;
    let b = block(&Frame::WORLD, [20.0, 10.0, 10.0])?.difference(&slot)?;
    b.delete_faces(&[b.face_at([8.0, 2.0, 5.0])?, b.face_at([12.0, 2.0, 5.0])?, b.face_at([10.0, 7.0, 5.0])?])
}
