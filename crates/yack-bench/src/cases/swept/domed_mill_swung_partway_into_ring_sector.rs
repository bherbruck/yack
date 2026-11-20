// yack: brep-bench case swept/domed-mill-swung-partway-into-ring-sector
//! A domed mill swung partway into a curved block.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let z = [0.0, 0.0, 1.0];
    let sector = revolve(&Frame::WORLD, &Profile::rectangle([8.0, -1.0], [12.0, 1.0])?, deg(60.0))?;
    let side = Loop::starting_at([0.0, -0.8]).arc_ccw_to([1.0, 1.2 - 3f64.sqrt()], [0.0, 1.2]).line_to([1.0, 3.0]);
    let start = Frame::at([10.0, 0.0, 0.0]).moved(&Rigid::rotation([0.0; 3], z, deg(-140.0))?);
    let mill = revolve(&start, &Profile::new(side.line_to([0.0, 3.0]).close()?, vec![])?, deg(360.0))?;
    sector.difference(&mill.swept(&Motion::Turn { axis_point: [0.0; 3].into(), axis: z.into(), angle: deg(170.0) })?)
}
