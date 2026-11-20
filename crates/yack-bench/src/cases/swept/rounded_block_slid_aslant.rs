// yack: brep-bench case swept/rounded-block-slid-aslant
//! A block with every edge rounded, slid aslant.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let b = block(&Frame::WORLD, [10.0, 6.0, 4.0])?;
    b.fillet(&b.edges(), 1.0)?.swept(&Motion::Slide { by: [8.0, 5.0, 3.0].into() })
}
