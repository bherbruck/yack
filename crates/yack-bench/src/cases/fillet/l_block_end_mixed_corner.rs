// yack: brep-bench case fillet/l-block-end-mixed-corner
//! An L-block's end rounded across its inside corner, one concave round and two convex.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let l = block(&Frame::WORLD, [20.0, 10.0, 6.0])?.union(&block(&Frame::WORLD, [6.0, 10.0, 20.0])?)?;
    let l = l.fillet(&[l.edge_at([6.0, 5.0, 6.0])?], 2.0)?;
    l.fillet(&[l.edge_at([6.0, 0.0, 14.0])?], 2.0)
}
