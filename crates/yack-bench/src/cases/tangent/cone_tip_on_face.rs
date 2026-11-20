// yack: brep-bench case tangent/cone-tip-on-face
//! A cone standing on its tip inside a block.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    block(&Frame::at([-10.0, -10.0, 0.0]), [20.0, 20.0, 10.0])?.union(&cone(&Frame::WORLD, 0.0, 5.0, 15.0)?)
}
