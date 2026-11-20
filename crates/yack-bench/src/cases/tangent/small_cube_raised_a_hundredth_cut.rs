// yack: brep-bench case tangent/small-cube-raised-a-hundredth-cut
//! A small cube less itself raised a hundredth of its side.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let cube = block(&Frame::WORLD, [0.0015, 0.0015, 0.0015])?;
    cube.difference(&block(&Frame::at([0.0, 0.0, 0.000015]), [0.0015, 0.0015, 0.0015])?)
}
