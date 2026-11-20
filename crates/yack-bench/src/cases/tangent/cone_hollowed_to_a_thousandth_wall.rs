// yack: brep-bench case tangent/cone-hollowed-to-a-thousandth-wall
//! A cone of base radius 5 hollowed to a wall a thousandth thick at its base.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    cone(&Frame::WORLD, 5.0, 0.0, 10.0)?.difference(&cone(&Frame::WORLD, 4.999, 0.0, 10.0)?)
}
