// yack: brep-bench case tangent/inscribed-ball-common
//! A cube and its inscribed ball, their common part.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    block(&Frame::WORLD, [10.0, 10.0, 10.0])?.intersection(&sphere(&Frame::at([5.0, 5.0, 5.0]), 5.0)?)
}
