//! A cube with every edge chamfered.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let cube = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    cube.chamfer(&cube.edges(), 2.0)
}
