// yack: brep-bench case fillet/slab-half-round
//! A block whose top is rounded into a half-cylinder.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let s = block(&Frame::WORLD, [10.0, 4.0, 6.0])?;
    s.fillet(&[s.edge_at([5.0, 0.0, 6.0])?, s.edge_at([5.0, 4.0, 6.0])?], 2.0)
}
