//! A cone shelled open at its base, its apex moved in.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let c = cone(&Frame::WORLD, 6.0, 0.0, 8.0)?;
    c.shell(&[c.face_at([1.0, 1.0, 0.0])?], 1.0)
}
