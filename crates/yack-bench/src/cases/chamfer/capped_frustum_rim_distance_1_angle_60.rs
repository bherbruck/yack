// yack: brep-bench case chamfer/capped-frustum-rim-distance-1-angle-60
//! A frustum capped by a ball, its rim chamfered by a distance of 1 and an angle of 60°.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let cap = sphere(&Frame::WORLD, 5.0)?.split(&Plane::new([0.0, 0.0, 3.0], [0.0, 0.0, 1.0])?)?.front;
    let s = cone(&Frame::WORLD, 2.0, 4.0, 3.0)?.union(&cap)?;
    s.chamfer_angle(s.edge_at([8f64.sqrt(), 8f64.sqrt(), 3.0])?, s.face_at([0.0, 3.0, 1.5])?, 1.0, deg(60.0))
}
