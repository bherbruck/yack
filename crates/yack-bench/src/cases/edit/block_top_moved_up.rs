// yack: brep-bench case edit/block-top-moved-up
//! A drilled block's top moved up.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let b = block(&Frame::WORLD, [40.0, 30.0, 10.0])?;
    let b = b.difference(&cylinder(&Frame::at([20.0, 15.0, -1.0]), 5.0, 12.0)?)?;
    b.move_faces(&[b.face_at([5.0, 5.0, 10.0])?], &Rigid::translation([0.0, 0.0, 6.0]))
}
