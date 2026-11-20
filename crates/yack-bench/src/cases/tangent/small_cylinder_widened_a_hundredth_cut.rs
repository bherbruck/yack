// yack: brep-bench case tangent/small-cylinder-widened-a-hundredth-cut
//! A copy a hundredth wider less a small cylinder.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    cylinder(&Frame::WORLD, 0.0007575, 0.0015)?.difference(&cylinder(&Frame::WORLD, 0.00075, 0.0015)?)
}
