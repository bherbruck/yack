// yack: brep-bench case chamfer/bored-ball-rim-two-distances
//! A bored ball whose rim is chamfered with two distances.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let s = sphere(&Frame::WORLD, 5.0)?.difference(&cylinder(&Frame::at([0.0, 0.0, -6.0]), 2.0, 12.0)?)?;
    s.chamfer_two(s.edge_at([2f64.sqrt(), 2f64.sqrt(), 21f64.sqrt()])?, s.face_at([0.0, 2.0, 0.0])?, 2.0, 1.0)
}
