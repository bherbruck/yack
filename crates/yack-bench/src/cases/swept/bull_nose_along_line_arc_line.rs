// yack: brep-bench case swept/bull-nose-along-line-arc-line
//! A bull-nose mill run along a line, an arc and a line.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let mill = cylinder(&Frame::at([0.0, 0.0, -1.5]), 2.0, 6.5)?;
    let mill = mill.fillet(&[mill.edge_at([0.0, 2.0, -1.5])?], 0.5)?;
    let turn = Motion::Turn { axis_point: [20.0, 10.0, 0.0].into(), axis: [0.0, 0.0, 1.0].into(), angle: deg(90.0) };
    let (on_x, on_y) = (Motion::Slide { by: [20.0, 0.0, 0.0].into() }, Motion::Slide { by: [0.0, 20.0, 0.0].into() });
    let groove = mill.swept(&Motion::Then(vec![on_x, turn, on_y]))?;
    block(&Frame::at([-10.0, -10.0, -10.0]), [50.0, 50.0, 10.0])?.difference(&groove)
}
