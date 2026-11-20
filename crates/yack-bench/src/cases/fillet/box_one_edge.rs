//! A cube with one edge rounded.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let cube = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    cube.fillet(&[cube.edge_at([10.0, 10.0, 5.0])?], 2.0)
}
