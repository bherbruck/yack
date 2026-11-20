// yack: brep-bench case singular/cone-halved-through-its-apex
//! A cone halved through its apex.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    Ok(cone(&Frame::WORLD, 5.0, 0.0, 10.0)?.split(&Plane::new([0.0, 0.0, 0.0], [1.0, 0.0, 0.0])?)?.front)
}
