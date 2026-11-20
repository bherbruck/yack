// yack: brep-bench case parts/v-belt-pulley
//! A V-belt pulley.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let hub = cylinder(&Frame::at([0.0, 0.0, -14.0]), 12.0, 28.0)?;
    let body = hub.union(&cylinder(&Frame::at([0.0, 0.0, -9.0]), 28.0, 18.0)?)?;
    let body = body.difference(&cylinder(&Frame::at([0.0, 0.0, -15.0]), 6.0, 30.0)?)?;
    let groove = revolve(&Frame::WORLD, &Profile::polygon(&[[22.0, 0.0], [29.0, -5.0], [29.0, 5.0]])?, deg(360.0))?;
    body.difference(&groove)
}
