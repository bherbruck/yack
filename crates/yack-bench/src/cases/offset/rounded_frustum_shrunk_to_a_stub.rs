//! A rounded cone frustum shrunk to a stub.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let (s, h) = (cone(&Frame::WORLD, 6.0, 3.0, 6.0)?, 1.5 * 2f64.sqrt());
    s.fillet(&[s.edge_at([2.0 * h, 2.0 * h, 0.0])?, s.edge_at([h, h, 6.0])?], 1.5)?.offset(-2.5)
}
