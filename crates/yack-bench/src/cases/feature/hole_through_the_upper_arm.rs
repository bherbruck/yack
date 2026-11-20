// yack: brep-bench case feature/hole-through-the-upper-arm
//! A hole drilled through the upper arm only.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let c = [[0.0, 0.0], [40.0, 0.0], [40.0, 6.0], [6.0, 6.0], [6.0, 24.0], [40.0, 24.0], [40.0, 30.0], [0.0, 30.0]];
    let xz = Frame::new([0.0, 10.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0])?;
    let bracket = extrude(&xz, &Profile::polygon(&c)?, 20.0)?;
    bracket.difference(&cylinder(&Frame::at([28.0, 0.0, 24.0]), 4.0, 7.0)?)
}
