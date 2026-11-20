//! A rounded block with its top rim rounded.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let b = block(&Frame::at([-10.0, -6.0, 0.0]), [20.0, 12.0, 5.0])?;
    let corners = [[10.0, 6.0, 2.5], [-10.0, 6.0, 2.5], [-10.0, -6.0, 2.5], [10.0, -6.0, 2.5]];
    let b = b.fillet(&corners.iter().map(|&p| b.edge_at(p)).collect::<Result<Vec<_>>>()?, 3.0)?;
    b.fillet(&[b.edge_at([0.0, 6.0, 5.0])?], 1.0)
}
