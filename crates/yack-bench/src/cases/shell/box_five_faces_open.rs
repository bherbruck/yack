//! A box shelled open at every face but one.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let b = block(&Frame::WORLD, [10.0, 8.0, 6.0])?;
    let open = [[5.0, 4.0, 6.0], [5.0, 0.0, 3.0], [5.0, 8.0, 3.0], [0.0, 4.0, 3.0], [10.0, 4.0, 3.0]];
    b.shell(&open.iter().map(|&p| b.face_at(p)).collect::<Result<Vec<_>>>()?, 1.0)
}
