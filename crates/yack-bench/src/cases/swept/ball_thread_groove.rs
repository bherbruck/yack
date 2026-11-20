// yack: brep-bench case swept/ball-thread-groove
//! A rod threaded by a ball.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let (o, z) = (Point3::from([0.0; 3]), Vec3::from([0.0, 0.0, 1.0]));
    let screw = Motion::screw(o, z, rad(0.0), deg(2160.0), Lead::per_turn(4.0));
    cylinder(&Frame::WORLD, 5.0, 20.0)?.cut_swept(&sphere(&Frame::at([5.0, 0.0, -2.0]), 1.0)?, &screw)
}
