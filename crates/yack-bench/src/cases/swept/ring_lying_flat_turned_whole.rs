// yack: brep-bench case swept/ring-lying-flat-turned-whole
//! A ring lying flat, turned a whole turn about a line beside it.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let turn = Motion::Turn { axis_point: [0.0; 3].into(), axis: [0.0, 0.0, 1.0].into(), angle: deg(360.0) };
    torus(&Frame::at([8.0, 0.0, 0.0]), 3.0, 1.0)?.swept(&turn)
}
