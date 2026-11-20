//! Two overlapping blocks, their common part.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let a = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    a.intersection(&block(&Frame::at([5.0, 5.0, 0.0]), [10.0, 10.0, 10.0])?)
}
