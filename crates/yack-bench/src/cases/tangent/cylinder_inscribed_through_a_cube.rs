// yack: brep-bench case tangent/cylinder-inscribed-through-a-cube
//! A cylinder inscribed through a cube.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    // seam at 45° to the lines of contact, along y = z
    let axis = Frame::along([-10.0, 0.0, 0.0], [1.0, 0.0, 0.0])?.turned(deg(45.0));
    block(&Frame::at([-5.0, -5.0, -5.0]), [10.0, 10.0, 10.0])?.union(&cylinder(&axis, 5.0, 20.0)?)
}
