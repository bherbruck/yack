// yack: brep-bench case shell/hole-skin-merging-wall
//! A drilled block shelled with its top open, the bore's tube running into a wall.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let bore = cylinder(&Frame::at([15.5, 10.0, -1.0]), 3.0, 12.0)?;
    let b = block(&Frame::WORLD, [20.0, 20.0, 10.0])?.difference(&bore)?;
    b.shell(&[b.face_at([1.0, 1.0, 10.0])?], 1.0)
}
