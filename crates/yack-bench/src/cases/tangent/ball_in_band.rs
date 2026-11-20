// yack: brep-bench case tangent/ball-in-band
//! A ball in a band that touches its equator.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    sphere(&Frame::WORLD, 5.0)?.union(&cylinder(&Frame::at([0.0, 0.0, -3.0]), 5.0, 6.0)?)
}
