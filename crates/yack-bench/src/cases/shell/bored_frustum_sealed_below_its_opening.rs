//! A bored cone frustum shelled open at its top, the cavity a ring sealed below it.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let bore = cylinder(&Frame::at([0.0, 0.0, -1.0]), 1.0, 6.0)?;
    let bored = cone(&Frame::WORLD, 6.0, 3.0, 4.0)?.difference(&bore)?;
    bored.shell(&[bored.face_at([0.0, 2.0, 4.0])?], 1.0)
}
