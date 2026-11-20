// yack: brep-bench case tangent/small-cube-raised-a-hundredth-common
//! A small cube and itself raised a hundredth of its side, their common part.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let cube = block(&Frame::WORLD, [0.0015, 0.0015, 0.0015])?;
    cube.intersection(&block(&Frame::at([0.0, 0.0, 0.000015]), [0.0015, 0.0015, 0.0015])?)
}
