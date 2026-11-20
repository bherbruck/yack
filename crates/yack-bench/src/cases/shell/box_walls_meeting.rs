//! A box shelled to half its width: the walls meet and nothing is hollowed.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let b = block(&Frame::WORLD, [10.0, 8.0, 6.0])?;
    b.shell(&[b.face_at([5.0, 4.0, 6.0])?], 4.0)
}
