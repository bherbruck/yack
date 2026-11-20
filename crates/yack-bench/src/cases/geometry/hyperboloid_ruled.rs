// yack: brep-bench case geometry/hyperboloid-ruled
//! A hyperboloid of one sheet as a ruled surface.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    // the top rim's frame is turned a quarter turn, so the ruling from angle θ below ends at θ + 90° above
    let rim = Profile::circle([0.0, 0.0], 5.0)?;
    ruled(&Frame::at([0.0, 0.0, -5.0]), &rim, &Frame::at([0.0, 0.0, 5.0]).turned(deg(90.0)), &rim)
}
