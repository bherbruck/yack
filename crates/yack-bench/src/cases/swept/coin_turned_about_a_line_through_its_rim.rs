// yack: brep-bench case swept/coin-turned-about-a-line-through-its-rim
//! A disc turned about an upright line through its rim.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let coin = cylinder(&Frame::new([2.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])?, 2.0, 0.5)?;
    coin.swept(&Motion::turn([0.0; 3], [0.0, 0.0, 1.0], rad(0.0), deg(90.0)))
}
