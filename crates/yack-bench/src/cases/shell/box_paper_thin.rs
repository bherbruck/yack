// yack: brep-bench case shell/box-paper-thin
//! A box shelled to a wall 0.001 thick.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let b = block(&Frame::WORLD, [10.0, 8.0, 6.0])?;
    b.shell(&[b.face_at([5.0, 4.0, 6.0])?], 0.001)
}
