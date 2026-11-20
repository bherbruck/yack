// yack: brep-bench case tangent/small-cone-widened-a-hundredth-cut
//! A copy a hundredth wider less a small cone.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    cone(&Frame::WORLD, 0.0007575, 0.0, 0.0015)?.difference(&cone(&Frame::WORLD, 0.00075, 0.0, 0.0015)?)
}
