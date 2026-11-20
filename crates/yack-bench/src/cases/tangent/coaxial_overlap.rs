// yack: brep-bench case tangent/coaxial-overlap
//! Two equal cylinders overlapping end to end.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    cylinder(&Frame::WORLD, 5.0, 10.0)?.union(&cylinder(&Frame::at([0.0, 0.0, 6.0]).turned(deg(90.0)), 5.0, 10.0)?)
}
