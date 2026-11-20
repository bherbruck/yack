//! Two separate rings linked through each other.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let second = Frame::new([3.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])?;
    torus(&Frame::WORLD, 3.0, 0.5)?.union(&torus(&second, 3.0, 0.5)?)
}
