// yack: brep-bench case swept/drill-swung-partway-into-ring-sector
//! A pointed drill swung partway into a curved block.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let (z, tip) = ([0.0, 0.0, 1.0], -0.8 + 1.0 / 59f64.to_radians().tan());
    let sector = revolve(&Frame::WORLD, &Profile::rectangle([8.0, -1.0], [12.0, 1.0])?, deg(60.0))?;
    let start = Frame::at([10.0, 0.0, 0.0]).moved(&Rigid::rotation([0.0; 3], z, deg(-140.0))?);
    let drill = revolve(&start, &Profile::polygon(&[[0.0, -0.8], [1.0, tip], [1.0, 3.0], [0.0, 3.0]])?, deg(360.0))?;
    sector.difference(&drill.swept(&Motion::Turn { axis_point: [0.0; 3].into(), axis: z.into(), angle: deg(170.0) })?)
}
