//! A notched cube with its inside corner rounded.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let notch = block(&Frame::at([10.0, 10.0, 10.0]), [10.0, 10.0, 10.0])?;
    let s = block(&Frame::WORLD, [20.0, 20.0, 20.0])?.difference(&notch)?;
    s.fillet(&[s.edge_at([15.0, 10.0, 10.0])?, s.edge_at([10.0, 15.0, 10.0])?, s.edge_at([10.0, 10.0, 15.0])?], 3.0)
}
