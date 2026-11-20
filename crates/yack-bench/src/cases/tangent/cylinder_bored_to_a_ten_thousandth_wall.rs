// yack: brep-bench case tangent/cylinder-bored-to-a-ten-thousandth-wall
//! A cylinder of radius 5 bored through to a wall a ten-thousandth thick.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    cylinder(&Frame::WORLD, 5.0, 10.0)?.difference(&cylinder(&Frame::WORLD, 4.9999, 10.0)?)
}
