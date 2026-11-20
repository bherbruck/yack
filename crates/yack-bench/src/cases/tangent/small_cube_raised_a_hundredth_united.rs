// yack: brep-bench case tangent/small-cube-raised-a-hundredth-united
//! A small cube united with itself raised a hundredth of its side.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let cube = block(&Frame::WORLD, [0.0015, 0.0015, 0.0015])?;
    cube.union(&block(&Frame::at([0.0, 0.0, 0.000015]), [0.0015, 0.0015, 0.0015])?)
}
