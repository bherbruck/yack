// yack: brep-bench case tangent/half-tori-overlapping
//! Two half tori of one torus, overlapping.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let top = torus(&Frame::WORLD, 10.0, 3.0)?.split(&Plane::new([0.0, 0.0, 0.0], [0.0, 0.0, 1.0])?)?.front;
    let side = torus(&Frame::WORLD.turned(deg(90.0)), 10.0, 3.0)?;
    top.union(&side.split(&Plane::new([0.0, 0.0, 0.0], [1.0, 0.0, 0.0])?)?.front)
}
