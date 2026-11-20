//! A lens of two balls.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    sphere(&Frame::WORLD, 5.0)?.intersection(&sphere(&Frame::at([6.0, 0.0, 0.0]), 5.0)?)
}
