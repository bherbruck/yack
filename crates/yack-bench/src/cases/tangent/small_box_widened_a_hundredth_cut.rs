// yack: brep-bench case tangent/small-box-widened-a-hundredth-cut
//! A copy a hundredth wider less a small square prism.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let copy = block(&Frame::at([-0.0007575, -0.0007575, 0.0]), [0.001515, 0.001515, 0.0015])?;
    copy.difference(&block(&Frame::at([-0.00075, -0.00075, 0.0]), [0.0015, 0.0015, 0.0015])?)
}
