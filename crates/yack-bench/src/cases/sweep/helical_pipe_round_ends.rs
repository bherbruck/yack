// yack: brep-bench case sweep/helical-pipe-round-ends
//! A helical pipe with rounded ends.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let ball = sphere(&Frame::at([10.0, 0.0, 0.0]), 1.0)?;
    let (axis_point, axis) = (Point3::ORIGIN, Vec3::new(0.0, 0.0, 1.0));
    ball.swept(&Motion::screw(axis_point, axis, rad(0.0), deg(1080.0), Lead::per_turn(5.0)))
}
