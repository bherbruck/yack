// yack: brep-bench case geometry/high-degree-bezier-dome
//! A block under a Bézier patch of degree 15.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let knots: Vec<f64> = [0.0; 16].into_iter().chain([1.0; 16]).collect();
    let row = |i: u32| (0..16).map(|j| f64::from(4 + (3 * i + 5 * j) % 7)).collect();
    slab(&Frame::WORLD, [20.0, 20.0], &knots, &knots, &(0..16).map(row).collect::<Vec<Vec<f64>>>())
}
