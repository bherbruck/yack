// yack: brep-bench case swept/v-bit-engraving-stopped
//! A V bit engraving a line that stops inside a block.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let bit = cone(&Frame::at([5.0, 0.0, -2.0]), 0.0, 5.0, 5.0)?;
    let groove = bit.swept(&Motion::slide([15.0, 0.0, 0.0]))?;
    block(&Frame::at([0.0, -10.0, -10.0]), [30.0, 20.0, 10.0])?.difference(&groove)
}
