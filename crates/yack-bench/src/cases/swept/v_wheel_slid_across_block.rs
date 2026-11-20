// yack: brep-bench case swept/v-wheel-slid-across-block
//! A V groove cut by a sharp V wheel slid across a block.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let t = 3.0 * 30f64.to_radians().tan();
    let hub = Frame::new([-8.0, 0.0, 14.0], [0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0])?;
    let wheel = revolve(&hub, &Profile::polygon(&[[3.0, -t], [6.0, 0.0], [3.0, t]])?, deg(360.0))?;
    let groove = wheel.swept(&Motion::slide([36.0, 0.0, 0.0]))?;
    block(&Frame::at([0.0, -5.0, 0.0]), [20.0, 10.0, 10.0])?.difference(&groove)
}
