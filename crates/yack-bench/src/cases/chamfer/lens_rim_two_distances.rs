// yack: brep-bench case chamfer/lens-rim-two-distances
//! A lens of two balls whose rim is chamfered with two distances.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let s = sphere(&Frame::at([0.0, 0.0, 3.0]), 5.0)?.intersection(&sphere(&Frame::at([0.0, 0.0, -3.0]), 5.0)?)?;
    s.chamfer_two(s.edge_at([0.0, 4.0, 0.0])?, s.face_at([0.0, 3.0, 1.0])?, 2.0, 1.0)
}
