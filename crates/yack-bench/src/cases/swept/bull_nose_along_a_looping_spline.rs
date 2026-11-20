// yack: brep-bench case swept/bull-nose-along-a-looping-spline
//! A groove that crosses itself: a mill run round a looping spline.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let plate = block(&Frame::at([-10.0, -10.0, -8.0]), [40.0, 35.0, 8.0])?;
    let path = Spline3::bezier(&[[0.0, 0.0, 0.0], [30.0, 20.0, 0.0], [-10.0, 20.0, 0.0], [20.0, 0.0, 0.0]])?;
    let mill = cylinder(&Frame::at([0.0, 0.0, -1.5]), 2.0, 6.5)?;
    let mill = mill.fillet(&[mill.edge_at([0.0, 2.0, -1.5])?], 0.5)?;
    plate.difference(&mill.swept(&Motion::Along { path, law: FrameLaw::Fixed })?)
}
