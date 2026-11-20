// yack: brep-bench case swept/torus-turned-across-block
//! A curved block grooved by a leaning cutter turned past it.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let (z, s, c) = ([0.0, 0.0, 1.0], 25f64.to_radians().sin(), 25f64.to_radians().cos());
    let sector = revolve(&Frame::WORLD, &Profile::rectangle([8.0, -1.0], [12.0, 1.0])?, deg(60.0))?;
    let lean = Frame::new([5.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c])?;
    let cutter = torus(&lean.moved(&Rigid::rotation([0.0; 3], z, deg(-140.0))?), 2.5, 2.0)?;
    sector.cut_swept(&cutter, &Motion::turn([0.0; 3], z, rad(0.0), deg(280.0)))
}
