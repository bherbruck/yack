// yack: brep-bench case swept/fine-thread-with-sharp-crests
//! A thread so fine its crests are sharp.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let (o, z) = (Point3::from([0.0; 3]), Vec3::from([0.0, 0.0, 1.0]));
    let start = Frame::at([4.5, -8.0, -0.45]).moved(&Rigid::rotation(o, z, deg(-180.0))?);
    let bar = block(&start.moved(&Rigid::translation([0.0, 0.0, -0.5])), [7.5, 16.0, 0.9])?;
    let screw = Motion::screw(o, z, rad(0.0), deg(7560.0), Lead::per_turn(1.0));
    cylinder(&Frame::WORLD, 5.0, 20.0)?.cut_swept(&bar, &screw)
}
