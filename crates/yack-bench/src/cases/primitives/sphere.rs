// yack: brep-bench case primitives/sphere
//! A ball.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    sphere(&Frame::WORLD, 5.0)
}
