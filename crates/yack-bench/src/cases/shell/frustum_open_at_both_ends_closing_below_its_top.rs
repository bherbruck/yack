//! A cone frustum shelled open at both ends, the cavity closing to a point below its top.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let f = cone(&Frame::WORLD, 6.0, 3.0, 4.0)?;
    f.shell(&[f.face_at([1.0, 1.0, 0.0])?, f.face_at([1.0, 1.0, 4.0])?], 2.7)
}
