//! A truncated cone.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    cone(&Frame::WORLD, 5.0, 2.0, 6.0)
}
