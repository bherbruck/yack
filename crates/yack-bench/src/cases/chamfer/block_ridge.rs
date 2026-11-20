// yack: brep-bench case chamfer/block-ridge
//! A block whose top is chamfered into a ridge.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let s = block(&Frame::WORLD, [10.0, 4.0, 6.0])?;
    s.chamfer(&[s.edge_at([5.0, 0.0, 6.0])?, s.edge_at([5.0, 4.0, 6.0])?], 2.0)
}
