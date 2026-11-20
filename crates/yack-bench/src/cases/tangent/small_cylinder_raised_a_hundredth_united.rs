// yack: brep-bench case tangent/small-cylinder-raised-a-hundredth-united
//! A small cylinder united with itself raised a hundredth of its height.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let c = cylinder(&Frame::WORLD, 0.00075, 0.0015)?;
    c.union(&cylinder(&Frame::at([0.0, 0.0, 0.000015]), 0.00075, 0.0015)?)
}
