// yack: brep-bench case offset/box-grown
//! A box grown by 1.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    block(&Frame::WORLD, [10.0, 8.0, 6.0])?.offset(1.0)
}
