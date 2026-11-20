// yack: brep-bench case boolean/box-minus-through-cylinder
//! A cube drilled through.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let cube = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    cube.difference(&cylinder(&Frame::at([5.0, 5.0, -1.0]), 3.0, 12.0)?)
}
