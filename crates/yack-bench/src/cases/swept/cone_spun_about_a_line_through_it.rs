// yack: brep-bench case swept/cone-spun-about-a-line-through-it
//! A cone spun about a line that crosses it aslant.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let (s, c) = (30f64.to_radians().sin(), 30f64.to_radians().cos());
    let base = Frame::new([5.0 * c, 0.0, 5.0 * s], [0.0, 1.0, 0.0], [s, 0.0, -c], [-c, 0.0, -s])?;
    let spike = cone(&base, 7.0 * 25f64.to_radians().tan(), 0.0, 7.0)?;
    spike.swept(&Motion::turn([0.0; 3], [0.0, 0.0, 1.0], rad(0.0), deg(150.0)))
}
