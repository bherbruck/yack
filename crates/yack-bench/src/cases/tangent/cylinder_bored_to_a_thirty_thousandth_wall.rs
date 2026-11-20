// yack: brep-bench case tangent/cylinder-bored-to-a-thirty-thousandth-wall
//! A cylinder of radius 5 bored through to a wall three hundred-thousandths thick.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    cylinder(&Frame::WORLD, 5.0, 10.0)?.difference(&cylinder(&Frame::WORLD, 4.99997, 10.0)?)
}
