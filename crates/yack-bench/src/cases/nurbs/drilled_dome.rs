// yack: brep-bench case nurbs/drilled-dome
//! The Bézier dome drilled through off center.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let k = [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
    let h = [[4.0, 6.0, 7.0, 5.0], [6.0, 14.0, 12.0, 7.0], [5.0, 13.0, 16.0, 8.0], [3.0, 6.0, 8.0, 6.0]].map(Vec::from);
    let dome = slab(&Frame::WORLD, [20.0, 20.0], &k, &k, &h)?;
    dome.difference(&cylinder(&Frame::at([12.0, 8.0, -1.0]), 3.0, 20.0)?)
}
