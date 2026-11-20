//! A box hollowed, its cavity closed.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    block(&Frame::WORLD, [10.0, 8.0, 6.0])?.shell(&[], 1.0)
}
