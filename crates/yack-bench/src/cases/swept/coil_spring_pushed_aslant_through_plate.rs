// yack: brep-bench case swept/coil-spring-pushed-aslant-through-plate
//! A coil spring pushed aslant through a plate.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let (o, z) = (Point3::from([0.0; 3]), Vec3::from([0.0, 0.0, 1.0]));
    let coil = Motion::screw(o, z, rad(0.0), deg(1080.0), Lead::per_turn(1.5));
    let spring = sphere(&Frame::at([2.0, 0.0, -12.0]), 0.4)?.swept(&coil)?;
    let plate = block(&Frame::at([-15.0, -15.0, 0.0]), [30.0, 30.0, 4.0])?;
    plate.cut_swept(&spring, &Motion::slide([3.0, 1.0, 18.0]))
}
