// yack: brep-bench case swept/end-mill-slid-across-block
//! A slot cut by an end mill slid across a block.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let slot = cylinder(&Frame::at([-5.0, 0.0, 7.0]), 2.0, 8.0)?.swept(&Motion::Slide { by: [30.0, 0.0, 0.0].into() })?;
    block(&Frame::at([0.0, -5.0, 0.0]), [20.0, 10.0, 10.0])?.difference(&slot)
}
