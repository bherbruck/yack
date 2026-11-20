//! A cylinder with its top rim rounded.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let s = cylinder(&Frame::WORLD, 5.0, 10.0)?;
    s.fillet(&[s.edge_at([0.0, 5.0, 10.0])?], 1.0)
}
