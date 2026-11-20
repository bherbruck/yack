// yack: brep-bench case boolean/box
//! A box.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    block(&Frame::WORLD, [1.0, 1.0, 1.0])
}
