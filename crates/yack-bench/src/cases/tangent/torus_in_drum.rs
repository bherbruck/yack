// yack: brep-bench case tangent/torus-in-drum
//! A torus through a drum that touches its outer equator.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    torus(&Frame::WORLD, 10.0, 3.0)?.union(&cylinder(&Frame::at([0.0, 0.0, -1.0]), 13.0, 2.0)?)
}
