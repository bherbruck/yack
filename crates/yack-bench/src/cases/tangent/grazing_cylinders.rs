// yack: brep-bench case tangent/grazing-cylinders
//! Two crossed cylinders barely overlapping.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let x = cylinder(&Frame::along([-15.0, 0.0, 0.0], [1.0, 0.0, 0.0])?, 5.0, 30.0)?;
    x.intersection(&cylinder(&Frame::along([0.0, -15.0, 9.99], [0.0, 1.0, 0.0])?, 5.0, 30.0)?)
}
