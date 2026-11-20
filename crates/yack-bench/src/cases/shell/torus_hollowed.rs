// yack: brep-bench case shell/torus-hollowed
//! A torus hollowed, its cavity closed.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    torus(&Frame::WORLD, 6.0, 2.0)?.shell(&[], 0.5)
}
