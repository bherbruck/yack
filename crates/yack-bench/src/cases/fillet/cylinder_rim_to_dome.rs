// yack: brep-bench case fillet/cylinder-rim-to-dome
//! A cylinder whose rim is rounded into a dome.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let s = cylinder(&Frame::WORLD, 5.0, 10.0)?;
    s.fillet(&[s.edge_at([0.0, 5.0, 10.0])?], 5.0)
}
