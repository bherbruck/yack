// yack: brep-bench case tangent/tangent-tee-common
//! A tangent tee, its common part.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let main = cylinder(&Frame::along([-15.0, 0.0, 0.0], [1.0, 0.0, 0.0])?, 5.0, 30.0)?;
    main.intersection(&cylinder(&Frame::along([0.0, -15.0, 2.0], [0.0, 1.0, 0.0])?, 3.0, 30.0)?)
}
