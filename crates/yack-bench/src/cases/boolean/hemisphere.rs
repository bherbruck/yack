//! A hemisphere.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    Ok(sphere(&Frame::WORLD, 5.0)?.split(&Plane::new([0.0, 0.0, 0.0], [0.0, 0.0, 1.0])?)?.front)
}
