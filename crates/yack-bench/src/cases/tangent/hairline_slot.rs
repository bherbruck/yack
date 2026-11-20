// yack: brep-bench case tangent/hairline-slot
//! A cube with a hairline saw cut.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let cube = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    cube.difference(&block(&Frame::at([5.0, -1.0, 5.0]), [0.000001, 12.0, 6.0])?)
}
