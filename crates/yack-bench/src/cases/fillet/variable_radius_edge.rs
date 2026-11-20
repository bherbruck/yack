// yack: brep-bench case fillet/variable-radius-edge
//! A cube edge rounded with a radius growing along it.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let cube = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    cube.fillet_variable(cube.edge_at([10.0, 10.0, 5.0])?, [10.0, 10.0, 0.0], 1.0, 2.0)
}
