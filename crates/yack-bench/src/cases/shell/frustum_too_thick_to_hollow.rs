//! A cone frustum shelled open at its top, so thick that nothing is hollowed.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let f = cone(&Frame::WORLD, 6.0, 3.0, 4.0)?;
    f.shell(&[f.face_at([1.0, 1.0, 4.0])?], 3.5)
}
