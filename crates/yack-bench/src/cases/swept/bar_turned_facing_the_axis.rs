// yack: brep-bench case swept/bar-turned-facing-the-axis
//! A curved block cut by a bar's side facing the axis squarely.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let (z, s, c) = ([0.0, 0.0, 1.0], 20f64.to_radians().sin(), 20f64.to_radians().cos());
    let sector = revolve(&Frame::WORLD, &Profile::rectangle([8.0, -1.0], [12.0, 1.0])?, deg(60.0))?;
    let corner = Frame::new([10.0 - 3.0 * s, -2.0, -3.0 * c], [c, 0.0, -s], [0.0, 1.0, 0.0], [s, 0.0, c])?;
    let bar = block(&corner.moved(&Rigid::rotation([0.0; 3], z, deg(-140.0))?), [3.0, 4.0, 6.0])?;
    sector.cut_swept(&bar, &Motion::turn([0.0; 3], z, rad(0.0), deg(280.0)))
}
