// yack: brep-bench case swept/bull-nose-pushed-in-at-a-lean
//! A bull-nose mill pushed straight down into a block at a lean.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let (s, c) = (30f64.to_radians().sin(), 30f64.to_radians().cos());
    let lean = Frame::new([0.0, 0.0, 4.0], [c, 0.0, -s], [0.0, 1.0, 0.0], [s, 0.0, c])?;
    let mill = cylinder(&lean, 2.0, 3.0)?;
    let mill = mill.fillet(&[mill.edge_at([0.0, 2.0, 4.0])?], 0.5)?;
    let hole = mill.swept(&Motion::slide([0.0, 0.0, -8.0]))?;
    block(&Frame::at([-8.0, -8.0, -8.0]), [16.0, 16.0, 8.0])?.difference(&hole)
}
