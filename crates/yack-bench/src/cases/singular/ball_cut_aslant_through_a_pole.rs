// yack: brep-bench case singular/ball-cut-aslant-through-a-pole
//! A ball cut aslant through its pole.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    Ok(sphere(&Frame::WORLD, 5.0)?.split(&Plane::new([5.0, 0.0, 0.0], [1.0, 0.0, 1.0])?)?.back)
}
