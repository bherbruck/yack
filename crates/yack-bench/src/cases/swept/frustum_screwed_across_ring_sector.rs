// yack: brep-bench case swept/frustum-screwed-across-ring-sector
//! A helical groove milled by a tapered cutter.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let z = [0.0, 0.0, 1.0];
    let sector = revolve(&Frame::WORLD, &Profile::rectangle([8.0, -1.0], [12.0, 1.0])?, deg(60.0))?;
    let start = Frame::at([10.0, 0.0, -0.6 - 2.0 * 140.0 / 360.0]).moved(&Rigid::rotation([0.0; 3], z, deg(-140.0))?);
    let screw = Motion::screw([0.0; 3], z, rad(0.0), deg(280.0), Lead::per_turn(2.0));
    sector.cut_swept(&cone(&start, 0.4, 1.0, 2.0)?, &screw)
}
