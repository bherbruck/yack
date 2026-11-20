// yack: brep-bench case shell/drilled-block-open-top
//! A drilled block shelled with its top open: the bore becomes a tube standing in the cup.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let bore = cylinder(&Frame::at([10.0, 10.0, -1.0]), 3.0, 12.0)?;
    let b = block(&Frame::WORLD, [20.0, 20.0, 10.0])?.difference(&bore)?;
    b.shell(&[b.face_at([1.0, 1.0, 10.0])?], 1.0)
}
