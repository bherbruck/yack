// yack: brep-bench case swept/drum-leaning-across-a-turn
//! A flat-end mill leaning across a turn, its rims digging past its ends.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let (s, c) = (25f64.to_radians().sin(), 25f64.to_radians().cos());
    let ring = revolve(&Frame::WORLD, &Profile::rectangle([3.0, -2.5], [8.0, 2.5])?, deg(360.0))?;
    let foot = Frame::new([5.0 - 1.5 * s, 1.2, -1.5 * c], [c, 0.0, -s], [0.0, 1.0, 0.0], [s, 0.0, c])?;
    let turn = Motion::Turn { axis_point: [0.0; 3].into(), axis: [0.0, 0.0, 1.0].into(), angle: deg(100.0) };
    ring.difference(&cylinder(&foot, 1.0, 3.0)?.swept(&turn)?)
}
