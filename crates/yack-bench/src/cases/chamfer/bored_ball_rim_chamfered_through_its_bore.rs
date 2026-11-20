// yack: brep-bench case chamfer/bored-ball-rim-chamfered-through-its-bore
//! A bored ball whose rim is chamfered the whole length of its bore.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let s = sphere(&Frame::WORLD, 5.0)?.difference(&cylinder(&Frame::at([0.0, 0.0, -6.0]), 2.0, 12.0)?)?;
    let (h, c) = (21f64.sqrt(), 2f64.sqrt());
    s.chamfer_two(s.edge_at([c, c, h])?, s.face_at([0.0, 2.0, 0.0])?, 2.0 * h, 1.0)
}
