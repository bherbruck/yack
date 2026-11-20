// yack: brep-bench case fillet/sliver-strip
//! A plate with a sliver of side left between two rounds.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let s = block(&Frame::WORLD, [20.0, 10.0, 2.0])?;
    s.fillet(&[s.edge_at([10.0, 10.0, 0.0])?, s.edge_at([10.0, 10.0, 2.0])?], 0.9999)
}
