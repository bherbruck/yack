// yack: brep-bench case tangent/ice-cream-cone
//! An ice-cream cone.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    sphere(&Frame::WORLD, 15.0)?.union(&cone(&Frame::at([0.0, 0.0, -25.0]), 0.0, 12.0, 16.0)?)
}
