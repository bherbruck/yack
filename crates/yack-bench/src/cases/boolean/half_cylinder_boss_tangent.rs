//! A cube with a half-round end.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let cube = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    cube.union(&cylinder(&Frame::at([10.0, 5.0, 0.0]), 5.0, 10.0)?)
}
