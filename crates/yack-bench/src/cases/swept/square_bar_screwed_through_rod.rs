// yack: brep-bench case swept/square-bar-screwed-through-rod
//! A rod threaded by a square bar screwed along it.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let (o, z) = (Point3::from([0.0; 3]), Vec3::from([0.0, 0.0, 1.0]));
    let start = Frame::at([4.0, -8.0, -1.0]).moved(&Rigid::rotation(o, z, deg(-90.0))?);
    let bar = block(&start.moved(&Rigid::translation([0.0, 0.0, -1.5])), [8.0, 16.0, 2.0])?;
    let screw = Motion::Screw { axis_point: o, axis: z, angle: deg(1440.0), rise_per_turn: 6.0 };
    cylinder(&Frame::WORLD, 5.0, 20.0)?.difference(&bar.swept(&screw)?)
}
