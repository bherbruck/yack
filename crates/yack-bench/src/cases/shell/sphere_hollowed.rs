// yack: brep-bench case shell/sphere-hollowed
//! A ball hollowed, its cavity closed.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    sphere(&Frame::WORLD, 5.0)?.shell(&[], 1.0)
}
