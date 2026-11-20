// yack: brep-bench case swept/half-drum-swung-partway-into-ring-sector
//! A half drum swung partway into a curved block, its arcs ending at its cut.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let z = [0.0, 0.0, 1.0];
    let sector = revolve(&Frame::WORLD, &Profile::rectangle([8.0, -1.0], [12.0, 1.0])?, deg(60.0))?;
    let drum = cylinder(&Frame::at([10.0, 0.0, -0.5]), 1.0, 1.0)?;
    let half = drum.split(&Plane::new([10.0, 0.0, 0.0], [0.0, -1.0, 0.0])?)?.front;
    let half = half.transform(&Rigid::rotation([0.0; 3], z, deg(-140.0))?.into())?;
    sector.cut_swept(&half, &Motion::turn([0.0; 3], z, rad(0.0), deg(170.0)))
}
