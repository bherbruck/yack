// yack: brep-bench case swept/ball-swung-groove
//! A groove swung by a ball.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let turn = Motion::turn([0.0; 3], [0.0, 0.0, 1.0], rad(0.0), deg(90.0));
    let groove = sphere(&Frame::at([12.0, 0.0, 0.0]), 3.0)?.swept(&turn)?;
    block(&Frame::at([-6.0, -6.0, -8.0]), [24.0, 24.0, 8.0])?.difference(&groove)
}
