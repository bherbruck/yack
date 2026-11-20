//! A knife edge of 5° rounded.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let blade = Profile::polygon(&[[0.0, 0.0], [10.0, 0.0], [10.0, 10.0 * 5f64.to_radians().tan()]])?;
    let s = extrude(&Frame::WORLD, &blade, 6.0)?;
    s.fillet(&[s.edge_at([0.0, 0.0, 3.0])?], 0.1)
}
