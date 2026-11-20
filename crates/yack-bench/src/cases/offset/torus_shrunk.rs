//! A torus shrunk by 1.5.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    torus(&Frame::WORLD, 10.0, 3.0)?.offset(-1.5)
}
