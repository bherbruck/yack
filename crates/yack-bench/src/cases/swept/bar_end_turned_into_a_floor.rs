// yack: brep-bench case swept/bar-end-turned-into-a-floor
//! A floor cut by a leaning bar's flat end, turned.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let (o, z) = (Point3::from([0.0; 3]), Vec3::from([0.0, 0.0, 1.0]));
    let (s, c) = (20f64.to_radians().sin(), 20f64.to_radians().cos());
    let sector = revolve(&Frame::WORLD, &Profile::rectangle([8.0, -1.0], [12.0, 1.0])?, deg(60.0))?;
    let (t, r) = (Rigid::rotation([10.0, 0.0, 0.0], [s, 0.0, c], deg(5.0))?, Rigid::rotation(o, z, deg(-140.0))?);
    let end = Frame::new([10.0, 0.0, 0.0], [c, 0.0, -s], [0.0, 1.0, 0.0], [s, 0.0, c])?.moved(&t).moved(&r);
    let bar = extrude(&end, &Profile::rectangle([0.0, -2.0], [3.0, 2.0])?, 3.0)?;
    sector.difference(&bar.swept(&Motion::Turn { axis_point: o, axis: z, angle: deg(280.0) })?)
}
