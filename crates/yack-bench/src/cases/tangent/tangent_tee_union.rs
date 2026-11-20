// yack: brep-bench case tangent/tangent-tee-union
//! A tee whose branch touches the main pipe's crown.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let main = cylinder(&Frame::along([-15.0, 0.0, 0.0], [1.0, 0.0, 0.0])?, 5.0, 30.0)?;
    main.union(&cylinder(&Frame::along([0.0, -15.0, 2.0], [0.0, 1.0, 0.0])?, 3.0, 30.0)?)
}
