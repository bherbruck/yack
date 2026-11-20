// yack: brep-bench case feature/pad-down-to-cylinder
//! A square pad extruded down to a cylinder.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let bar = cylinder(&Frame::along([-20.0, 0.0, 0.0], [1.0, 0.0, 0.0])?, 10.0, 40.0)?;
    bar.union(&block(&Frame::at([-3.0, -3.0, 0.0]), [6.0, 6.0, 15.0])?)
}
