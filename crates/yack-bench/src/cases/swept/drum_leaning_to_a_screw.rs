// yack: brep-bench case swept/drum-leaning-to-a-screw
//! A helical groove milled by a drum leaning to the screw's axis.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let (o, z) = (Point3::from([0.0; 3]), Vec3::from([0.0, 0.0, 1.0]));
    let (s, c) = (25f64.to_radians().sin(), 25f64.to_radians().cos());
    let sector = revolve(&Frame::WORLD, &Profile::rectangle([8.0, -1.0], [12.0, 1.0])?, deg(60.0))?;
    let foot = Frame::new([10.0, -1.5 * s, -1.5 * c], [1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c])?;
    let start = foot.moved(&Rigid::rotation(o, z, deg(-140.0))?).moved(&Rigid::translation([0.0, 0.0, -140.0 / 360.0]));
    let screw = Motion::Screw { axis_point: o, axis: z, angle: deg(280.0), rise_per_turn: 1.0 };
    sector.difference(&cylinder(&start, 0.8, 3.0)?.swept(&screw)?)
}
