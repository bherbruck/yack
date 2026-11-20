// yack: brep-bench case shell/drilled-block-open-at-its-bore
//! A drilled block shelled open at its bore.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let bore = cylinder(&Frame::at([10.0, 10.0, -1.0]), 3.0, 12.0)?;
    let b = block(&Frame::WORLD, [20.0, 20.0, 10.0])?.difference(&bore)?;
    b.shell(&[b.face_at([10.0, 7.0, 5.0])?], 1.0)
}
