// yack: brep-bench case swept/rod-spun-about-a-line-through-it
//! A rod spun partway about a line through it.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let rod = cylinder(&Frame::new([-3.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0])?, 0.5, 8.0)?;
    rod.swept(&Motion::Turn { axis_point: [0.0; 3].into(), axis: [0.0, 0.0, 1.0].into(), angle: deg(120.0) })
}
