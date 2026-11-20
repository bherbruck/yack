// yack: brep-bench case boolean/chain-of-three-balls
//! A chain of three balls.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let two = sphere(&Frame::WORLD, 5.0)?.union(&sphere(&Frame::at([7.0, 0.0, 0.0]), 4.0)?)?;
    two.union(&sphere(&Frame::at([10.0, 4.0, 0.0]), 3.0)?)
}
