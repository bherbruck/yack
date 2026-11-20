// yack: brep-bench case offset/turned-post-shrunk-past-its-round
//! A turned post with its shoulder rounded, shrunk by 1.5, past the round.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let s = cylinder(&Frame::WORLD, 5.0, 6.0)?.union(&cone(&Frame::at([0.0, 0.0, 6.0]), 5.0, 2.0, 4.0)?)?;
    let h = 2.5 * 2f64.sqrt();
    s.fillet(&[s.edge_at([h, h, 6.0])?], 1.0)?.offset(-1.5)
}
