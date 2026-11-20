// yack: brep-bench case chamfer/hemisphere-rim-chamfered-to-its-center
//! A hemisphere whose rim is chamfered until its flat face is gone.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let s = sphere(&Frame::WORLD, 5.0)?.split(&Plane::new([0.0, 0.0, 0.0], [0.0, 0.0, 1.0])?)?.front;
    s.chamfer_two(s.edge_at([0.0, 5.0, 0.0])?, s.face_at([1.0, 1.0, 0.0])?, 5.0, 1.0)
}
