// yack: brep-bench case tangent/small-cube-turned-about-its-edge-common
//! A small cube and itself turned 1.2° about one of its edges, their common part.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let cube = block(&Frame::WORLD, [0.001, 0.001, 0.001])?;
    cube.intersection(&block(&Frame::WORLD.turned(deg(1.2)), [0.001, 0.001, 0.001])?)
}
