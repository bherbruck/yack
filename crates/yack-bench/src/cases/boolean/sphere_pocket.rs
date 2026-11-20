//! A block with a spherical pocket.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    block(&Frame::at([-10.0, -10.0, -10.0]), [20.0, 20.0, 10.0])?.difference(&sphere(&Frame::WORLD, 5.0)?)
}
