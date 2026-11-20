// yack: brep-bench case swept/drill-pushed-through-plate
//! A hole bored by a drill pushed along its own axis.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let tip = 6.0 + 2.0 / 59f64.to_radians().tan();
    let point = Profile::polygon(&[[0.0, 6.0], [2.0, tip], [2.0, 14.0], [0.0, 14.0]])?;
    let drill = revolve(&Frame::at([10.0, 0.0, 0.0]), &point, deg(360.0))?;
    let hole = drill.swept(&Motion::Slide { by: [0.0, 0.0, -20.0].into() })?;
    block(&Frame::at([0.0, -5.0, 0.0]), [20.0, 10.0, 4.0])?.difference(&hole)
}
