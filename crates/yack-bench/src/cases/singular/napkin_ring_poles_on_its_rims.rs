// yack: brep-bench case singular/napkin-ring-poles-on-its-rims
//! A napkin ring with its ball's poles on its rims.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let ball = sphere(&Frame::along([0.0; 3], [3.0, 0.0, 4.0])?, 5.0)?;
    ball.difference(&cylinder(&Frame::at([0.0, 0.0, -6.0]), 3.0, 12.0)?)
}
