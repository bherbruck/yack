// yack: brep-bench case chamfer/ball-on-post-two-distances
//! A ball on a post, the concave edge between them chamfered with two distances.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let s = cylinder(&Frame::WORLD, 3.0, 10.0)?.union(&sphere(&Frame::at([0.0, 0.0, 10.0]), 5.0)?)?;
    s.chamfer_two(s.edge_at([1.5 * 2f64.sqrt(), 1.5 * 2f64.sqrt(), 6.0])?, s.face_at([0.0, 3.0, 3.0])?, 2.0, 1.0)
}
