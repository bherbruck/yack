//! A drilled block's hole deleted: the block is whole again.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let b = block(&Frame::WORLD, [20.0, 10.0, 10.0])?.difference(&cylinder(&Frame::at([10.0, 5.0, -1.0]), 3.0, 12.0)?)?;
    b.delete_faces(&[b.face_at([10.0, 8.0, 5.0])?])
}
