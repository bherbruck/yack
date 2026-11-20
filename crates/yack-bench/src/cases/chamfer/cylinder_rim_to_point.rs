// yack: brep-bench case chamfer/cylinder-rim-to-point
//! A cylinder whose rim is chamfered to a point.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let s = cylinder(&Frame::WORLD, 5.0, 10.0)?;
    s.chamfer(&[s.edge_at([0.0, 5.0, 10.0])?], 5.0)
}
