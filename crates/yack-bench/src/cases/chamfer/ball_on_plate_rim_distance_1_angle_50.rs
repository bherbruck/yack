// yack: brep-bench case chamfer/ball-on-plate-rim-distance-1-angle-50
//! A ball on a plate, the concave edge between them chamfered by a distance of 1 and an angle of
//! 50°.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let s = cylinder(&Frame::at([0.0, 0.0, -6.0]), 8.0, 6.0)?.union(&sphere(&Frame::WORLD, 5.0)?)?;
    s.chamfer_angle(s.edge_at([0.0, 5.0, 0.0])?, s.face_at([0.0, 6.5, 0.0])?, 1.0, deg(50.0))
}
