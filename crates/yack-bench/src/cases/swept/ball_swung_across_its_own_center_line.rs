// yack: brep-bench case swept/ball-swung-across-its-own-center-line
//! A ball swung about an axis that passes through it.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let turn = Motion::Turn { axis_point: [0.0; 3].into(), axis: [0.0, 0.0, 1.0].into(), angle: deg(120.0) };
    sphere(&Frame::at([1.0, 0.0, 0.0]), 2.0)?.swept(&turn)
}
