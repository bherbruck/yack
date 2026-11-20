//! A ridge nearly flat, rounded by a radius of 180.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let ridge = 5.0 + 4.0 * 0.5f64.to_radians().tan();
    let roof = Profile::polygon(&[[0.0, 0.0], [8.0, 0.0], [8.0, 5.0], [4.0, ridge], [0.0, 5.0]])?;
    let s = extrude(&Frame::WORLD, &roof, 6.0)?;
    s.fillet(&[s.edge_at([4.0, ridge, 3.0])?], 180.0)
}
