// yack: brep-bench case tangent/small-cylinder-slid-a-fiftieth-united
//! A small cylinder united with itself slid a fiftieth of its radius.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let c = cylinder(&Frame::WORLD, 0.00075, 0.0015)?;
    c.union(&cylinder(&Frame::at([0.000015, 0.0, 0.0]), 0.00075, 0.0015)?)
}
