//! A rounded cone frustum hollowed with walls thicker than its rounds.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let f = cone(&Frame::WORLD, 6.0, 3.0, 6.0)?;
    f.fillet(&[f.edge_at([0.0, -6.0, 0.0])?, f.edge_at([0.0, -3.0, 6.0])?], 1.5)?.shell(&[], 2.0)
}
