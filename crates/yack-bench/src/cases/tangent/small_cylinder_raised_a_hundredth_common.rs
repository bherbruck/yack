// yack: brep-bench case tangent/small-cylinder-raised-a-hundredth-common
//! A small cylinder and itself raised a hundredth of its height, their common part.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let c = cylinder(&Frame::WORLD, 0.00075, 0.0015)?;
    c.intersection(&cylinder(&Frame::at([0.0, 0.0, 0.000015]), 0.00075, 0.0015)?)
}
