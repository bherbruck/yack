// yack: brep-bench case tangent/torus-through-a-square-slab
//! A torus through a square slab, touching its four sides.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let ring = torus(&Frame::WORLD.turned(deg(45.0)), 10.0, 3.0)?;
    ring.union(&block(&Frame::at([-13.0, -13.0, -1.0]), [26.0, 26.0, 2.0])?)
}
