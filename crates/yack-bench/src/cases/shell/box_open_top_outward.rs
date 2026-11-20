//! A box shelled outward with its top open, its edges rounded.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let b = block(&Frame::WORLD, [10.0, 8.0, 6.0])?;
    let cup = b.offset(1.0)?.split(&Plane::new([0.0, 0.0, 6.0], [0.0, 0.0, 1.0])?)?.back;
    cup.difference(&b)
}
