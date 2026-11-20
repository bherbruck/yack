//! A tall cone frustum shelled open at its narrow top, the cavity a cone sealed well below it.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let f = cone(&Frame::WORLD, 7.0, 1.0, 8.0)?;
    f.shell(&[f.face_at([0.2, 0.2, 8.0])?], 2.0)
}
