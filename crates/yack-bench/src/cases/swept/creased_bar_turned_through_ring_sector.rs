// yack: brep-bench case swept/creased-bar-turned-through-ring-sector
//! A groove cut by a bar drafted both ways from a parting line.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let (s, c) = 20f64.to_radians().sin_cos();
    let plane = Frame::new([0.0; 3], [1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c])?;
    let bar = extrude_drafted(&plane, &Profile::rectangle([10.0, -2.0], [13.0, 2.0])?, 4.0, deg(2.0), true)?;
    let sector = revolve(&Frame::WORLD, &Profile::rectangle([8.0, -1.0], [12.0, 1.0])?, deg(60.0))?;
    sector.cut_swept(&bar, &Motion::turn([0.0; 3], [0.0, 0.0, 1.0], deg(-140.0), deg(140.0)))
}
