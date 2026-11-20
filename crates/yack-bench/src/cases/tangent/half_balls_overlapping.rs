// yack: brep-bench case tangent/half-balls-overlapping
//! Two half balls of one ball, overlapping.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let top = sphere(&Frame::WORLD, 5.0)?.split(&Plane::new([0.0, 0.0, 0.0], [0.0, 0.0, 1.0])?)?.front;
    let side = sphere(&Frame::along([0.0, 0.0, 0.0], [1.0, 0.0, 0.0])?, 5.0)?;
    top.union(&side.split(&Plane::new([0.0, 0.0, 0.0], [1.0, 0.0, 0.0])?)?.front)
}
