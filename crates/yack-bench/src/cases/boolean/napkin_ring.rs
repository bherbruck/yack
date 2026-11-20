//! A napkin ring.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    sphere(&Frame::WORLD, 5.0)?.difference(&cylinder(&Frame::at([0.0, 0.0, -6.0]), 3.0, 12.0)?)
}
