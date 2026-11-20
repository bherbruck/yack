//! A cone frustum with its top rim rounded, shrunk past the round's radius.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let s = cone(&Frame::WORLD, 6.0, 3.0, 6.0)?;
    let h = 1.5 * 2f64.sqrt();
    s.fillet(&[s.edge_at([h, h, 6.0])?], 1.5)?.offset(-2.0)
}
