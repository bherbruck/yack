// yack: brep-bench case tangent/bore-a-micron-from-the-wall
//! A tube bored off center, its wall a micron thin on one side.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    cylinder(&Frame::WORLD, 5.0, 10.0)?.difference(&cylinder(&Frame::at([0.999999, 0.0, -1.0]), 4.0, 12.0)?)
}
