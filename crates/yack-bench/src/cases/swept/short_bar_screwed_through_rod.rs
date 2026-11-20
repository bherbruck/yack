// yack: brep-bench case swept/short-bar-screwed-through-rod
//! A rod threaded by a short bar whose corners cut.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let (o, z) = (Point3::from([0.0; 3]), Vec3::from([0.0, 0.0, 1.0]));
    let start = Frame::at([4.0, -1.5, -1.0]).moved(&Rigid::rotation(o, z, deg(-90.0))?);
    let bar = block(&start.moved(&Rigid::translation([0.0, 0.0, -1.5])), [8.0, 3.0, 2.0])?;
    let screw = Motion::screw(o, z, rad(0.0), deg(1440.0), Lead::per_turn(6.0));
    cylinder(&Frame::WORLD, 5.0, 20.0)?.cut_swept(&bar, &screw)
}
