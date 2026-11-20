// yack: brep-bench case primitives/box
//! A box.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    block(&Frame::WORLD, [10.0, 20.0, 30.0])
}
