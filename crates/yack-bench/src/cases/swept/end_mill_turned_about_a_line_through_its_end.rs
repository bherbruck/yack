// yack: brep-bench case swept/end-mill-turned-about-a-line-through-its-end
//! A pocket cut by an end mill turned about a line through its flat end.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let turn = Motion::turn([0.0; 3], [0.0, 0.0, 1.0], rad(0.0), deg(120.0));
    let mill = cylinder(&Frame::at([1.5, 0.0, -2.0]), 3.0, 6.0)?;
    block(&Frame::at([-10.0, -10.0, -5.0]), [20.0, 20.0, 5.0])?.cut_swept(&mill, &turn)
}
