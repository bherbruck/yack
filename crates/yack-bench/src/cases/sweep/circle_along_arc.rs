// yack: brep-bench case sweep/circle-along-arc
//! A circle swept along an arc.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let start = Frame::new([10.0, 0.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0])?;
    sweep(&start, &Profile::circle([0.0, 0.0], 2.0)?, &[PathPiece::Arc { radius: 10.0, angle: deg(90.0) }])
}
