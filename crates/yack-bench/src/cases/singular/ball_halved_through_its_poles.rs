// yack: brep-bench case singular/ball-halved-through-its-poles
//! A ball halved through its poles.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    Ok(sphere(&Frame::WORLD, 5.0)?.split(&Plane::new([0.0, 0.0, 0.0], [1.0, 0.0, 0.0])?)?.front)
}
