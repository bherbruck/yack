// yack: brep-bench case primitives/torus
//! A torus.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    torus(&Frame::WORLD, 10.0, 2.0)
}
