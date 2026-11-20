// yack: brep-bench case swept/ball-on-a-leaning-stem-swung-partway
//! A ball on a leaning stem swung partway into a block.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let (z, s, c) = ([0.0, 0.0, 1.0], 20f64.to_radians().sin(), 20f64.to_radians().cos());
    let sector = revolve(&Frame::WORLD, &Profile::rectangle([8.0, -1.0], [12.0, 1.0])?, deg(60.0))?;
    let stem = cylinder(&Frame::new([10.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c])?, 0.4, 3.0)?;
    let tool = sphere(&Frame::at([10.0, 0.0, 0.0]), 0.8)?.union(&stem)?;
    let tool = tool.transform(&Rigid::rotation([0.0; 3], z, deg(-140.0))?.into())?;
    sector.difference(&tool.swept(&Motion::Turn { axis_point: [0.0; 3].into(), axis: z.into(), angle: deg(170.0) })?)
}
