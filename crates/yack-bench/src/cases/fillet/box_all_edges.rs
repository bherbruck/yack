// yack: brep-bench case fillet/box-all-edges
//! A cube with every edge rounded.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let cube = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    cube.fillet(&cube.edges(), 1.0)
}
