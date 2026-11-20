// yack: brep-bench case sweep/helical-pipe-round-ends
//! A helical pipe with rounded ends.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let ball = sphere(&Frame::at([10.0, 0.0, 0.0]), 1.0)?;
    let (axis_point, axis) = ([0.0; 3].into(), [0.0, 0.0, 1.0].into());
    ball.swept(&Motion::Screw { axis_point, axis, angle: deg(1080.0), rise_per_turn: 5.0 })
}
