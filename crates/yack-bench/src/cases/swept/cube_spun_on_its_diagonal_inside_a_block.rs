// yack: brep-bench case swept/cube-spun-on-its-diagonal-inside-a-block
//! A void spun by a cube turned about its long diagonal inside a block.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let spin = Motion::Turn { axis_point: [-1.0, -1.0, -1.0].into(), axis: [1.0, 1.0, 1.0].into(), angle: deg(360.0) };
    let void = block(&Frame::at([-1.0, -1.0, -1.0]), [2.0, 2.0, 2.0])?.swept(&spin)?;
    block(&Frame::at([-3.0, -3.0, -3.0]), [6.0, 6.0, 6.0])?.difference(&void)
}
