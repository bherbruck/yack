// yack: brep-bench case tangent/inscribed-ball-union
//! A cube united with its inscribed ball.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    block(&Frame::WORLD, [10.0, 10.0, 10.0])?.union(&sphere(&Frame::at([5.0, 5.0, 5.0]), 5.0)?)
}
