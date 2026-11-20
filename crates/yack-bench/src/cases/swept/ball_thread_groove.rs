// yack: brep-bench case swept/ball-thread-groove
//! A rod threaded by a ball.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let (o, z) = (Point3::from([0.0; 3]), Vec3::from([0.0, 0.0, 1.0]));
    let screw = Motion::Screw { axis_point: o, axis: z, angle: deg(2160.0), rise_per_turn: 4.0 };
    cylinder(&Frame::WORLD, 5.0, 20.0)?.difference(&sphere(&Frame::at([5.0, 0.0, -2.0]), 1.0)?.swept(&screw)?)
}
