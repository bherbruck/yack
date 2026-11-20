// yack: brep-bench case tangent/pole-cap-a-ten-thousandth-high
//! A ball's cap a ten-thousandth high, around its pole.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    sphere(&Frame::WORLD, 5.0)?.intersection(&block(&Frame::at([-6.0, -6.0, 4.9999]), [12.0, 12.0, 2.0])?)
}
