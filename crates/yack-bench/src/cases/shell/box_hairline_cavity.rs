// yack: brep-bench case shell/box-hairline-cavity
//! A box shelled nearly to half its width, leaving a cavity 0.02 wide.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let b = block(&Frame::WORLD, [10.0, 8.0, 6.0])?;
    b.shell(&[b.face_at([5.0, 4.0, 6.0])?], 3.99)
}
