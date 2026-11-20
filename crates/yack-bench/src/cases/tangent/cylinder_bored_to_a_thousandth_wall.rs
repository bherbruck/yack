// yack: brep-bench case tangent/cylinder-bored-to-a-thousandth-wall
//! A cylinder of radius 5 bored through to a wall a thousandth thick.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    cylinder(&Frame::WORLD, 5.0, 10.0)?.difference(&cylinder(&Frame::WORLD, 4.999, 10.0)?)
}
