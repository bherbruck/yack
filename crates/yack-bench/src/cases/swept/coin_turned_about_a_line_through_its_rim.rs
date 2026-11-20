// yack: brep-bench case swept/coin-turned-about-a-line-through-its-rim
//! A disc turned about an upright line through its rim.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let coin = cylinder(&Frame::new([2.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])?, 2.0, 0.5)?;
    coin.swept(&Motion::Turn { axis_point: [0.0; 3].into(), axis: [0.0, 0.0, 1.0].into(), angle: deg(90.0) })
}
