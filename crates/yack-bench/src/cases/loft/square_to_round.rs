// yack: brep-bench case loft/square-to-round
//! A square-to-round transition.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let square = Profile::rectangle([-20.0, -20.0], [20.0, 20.0])?;
    loft_through(&Frame::WORLD, &[(0.0, square), (30.0, Profile::circle([0.0, 0.0], 15.0)?)])
}
