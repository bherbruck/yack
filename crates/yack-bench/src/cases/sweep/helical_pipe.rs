// yack: brep-bench case sweep/helical-pipe
//! A helical pipe.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    // the section square to the helix where it starts: its tangent at (10, 0, 0) is (0, 2π·10, 5)
    let start = Frame::along([10.0, 0.0, 0.0], [0.0, 20.0 * std::f64::consts::PI, 5.0])?;
    sweep_helix(&start, &Profile::circle([0.0, 0.0], 1.0)?, [0.0; 3], [0.0, 0.0, 1.0], Lead::per_turn(5.0), deg(1080.0))
}
