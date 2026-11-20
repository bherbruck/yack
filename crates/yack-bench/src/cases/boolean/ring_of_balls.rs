// yack: brep-bench case boolean/ring-of-balls
//! Twelve balls on a ring, each overlapping its neighbors.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    (0..12).try_fold(empty(), |ring, i| {
        let a = (30.0 * f64::from(i)).to_radians();
        ring.union(&sphere(&Frame::at([10.0 * a.cos(), 10.0 * a.sin(), 0.0]), 3.0)?)
    })
}
