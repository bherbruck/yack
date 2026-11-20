// yack: brep-bench case loft/square-to-round
//! A square-to-round transition.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    // straight guides from each corner of the square to the two circle points beside it (the cones' edge lines)
    let a = [([20., -20.], [0., -15.]), ([20., -20.], [15., 0.]), ([20., 20.], [15., 0.])];
    let b = [([20., 20.], [0., 15.]), ([-20., 20.], [0., 15.]), ([-20., 20.], [-15., 0.])];
    let c = [([-20., -20.], [-15., 0.]), ([-20., -20.], [0., -15.])];
    let lines = a.iter().chain(&b).chain(&c).map(|([x, y], [u, v])| Spline3::bezier(&[[*x, *y, 0.0], [*u, *v, 30.0]]));
    let (guides, top) = (lines.collect::<Result<Vec<_>>>()?, Profile::circle([0.0, 0.0], 15.0)?);
    let foot = (Frame::WORLD, Profile::rectangle([-20.0, -20.0], [20.0, 20.0])?);
    loft_guided(&[foot, (Frame::at([0.0, 0.0, 30.0]), top)], &guides)
}
