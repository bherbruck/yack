// yack: brep-bench case chamfer/hemisphere-rim-distance-2-angle-45
//! A hemisphere's rim chamfered by a distance of 2 and an angle of 45°.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let s = sphere(&Frame::WORLD, 5.0)?.split(&Plane::new([0.0, 0.0, 0.0], [0.0, 0.0, 1.0])?)?.front;
    s.chamfer_angle(s.edge_at([0.0, 5.0, 0.0])?, s.face_at([1.0, 1.0, 0.0])?, 2.0, deg(45.0))
}
