// yack: brep-bench case boolean/tube
//! A tube.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    cylinder(&Frame::WORLD, 5.0, 10.0)?.difference(&cylinder(&Frame::at([0.0, 0.0, -1.0]), 3.0, 12.0)?)
}
