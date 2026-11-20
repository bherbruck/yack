// yack: brep-bench case fillet/conic-edge
//! A block's edge blended by a parabolic conic.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let s = block(&Frame::WORLD, [20.0, 10.0, 10.0])?;
    s.fillet_conic(&[s.edge_at([20.0, 5.0, 10.0])?], 3.0, 0.5)
}
