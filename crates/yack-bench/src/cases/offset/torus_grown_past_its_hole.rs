//! A torus grown past its hole.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    torus(&Frame::WORLD, 4.0, 2.0)?.offset(3.0)
}
