// yack: brep-bench case fillet/bezier-dome-one-edge
//! The Bézier dome block with one top edge rounded.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let k = [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
    let z = [[4.0, 6.0, 7.0, 5.0], [6.0, 14.0, 12.0, 7.0], [5.0, 13.0, 16.0, 8.0], [3.0, 6.0, 8.0, 6.0]].map(Vec::from);
    let s = slab(&Frame::WORLD, [20.0, 20.0], &k, &k, &z)?;
    s.fillet(&[s.edge_at([10.0, 0.0, 5.0])?], 1.0)
}
