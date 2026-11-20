// yack: brep-bench case swept/bar-swung-partway-into-ring-sector
//! A small bar swung partway into a curved block.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let z = [0.0, 0.0, 1.0];
    let sector = revolve(&Frame::WORLD, &Profile::rectangle([8.0, -1.0], [12.0, 1.0])?, deg(60.0))?;
    let start = Frame::at([10.0, -0.5, -0.6]).moved(&Rigid::rotation([0.0; 3], z, deg(-140.0))?);
    let bar = block(&start, [1.0, 1.0, 1.2])?;
    sector.difference(&bar.swept(&Motion::Turn { axis_point: [0.0; 3].into(), axis: z.into(), angle: deg(170.0) })?)
}
