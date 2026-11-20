//! A cone frustum with its bottom rim rounded, shelled open at its top past the round's radius.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let f = cone(&Frame::WORLD, 6.0, 3.0, 6.0)?;
    let f = f.fillet(&[f.edge_at([0.0, -6.0, 0.0])?], 1.5)?;
    f.shell(&[f.face_at([1.0, 1.0, 6.0])?], 2.0)
}
