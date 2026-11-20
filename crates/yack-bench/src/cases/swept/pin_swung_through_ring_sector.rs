// yack: brep-bench case swept/pin-swung-through-ring-sector
//! A slot swung by a pin lying along a radius.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let z = [0.0, 0.0, 1.0];
    let sector = revolve(&Frame::WORLD, &Profile::rectangle([8.0, -1.0], [12.0, 1.0])?, deg(60.0))?;
    let end = Frame::new([6.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0])?;
    let pin = cylinder(&end.moved(&Rigid::rotation([0.0; 3], z, deg(-140.0))?), 0.5, 5.0)?;
    sector.cut_swept(&pin, &Motion::turn([0.0; 3], z, rad(0.0), deg(280.0)))
}
