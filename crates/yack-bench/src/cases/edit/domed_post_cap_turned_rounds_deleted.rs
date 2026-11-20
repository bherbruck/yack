// yack: brep-bench case edit/domed-post-cap-turned-rounds-deleted
//! A post capped by a ball, the cap made turned 1.5 radians about the axis, its rounded rim
//! deleted: the cylinder and the ball meet again.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let (c, s) = (1.5f64.cos(), 1.5f64.sin());
    let ball = sphere(&Frame::new([0.0, 0.0, 3.0], [c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0])?, 5.0)?;
    let dome = ball.split(&Plane::new([0.0, 0.0, 6.0], [0.0, 0.0, 1.0])?)?.front;
    let post = cylinder(&Frame::WORLD, 4.0, 6.0)?.union(&dome)?;
    let rim = post.edges_between(post.face_at([0.0, 4.0, 3.0])?, post.face_at([0.0, 3.0, 7.0])?)?;
    let rounded = post.fillet(&rim, 1.0)?;
    rounded.delete_faces(&rounded.faces_where(|f| matches!(f.kind(), yack::SurfaceKind::Torus))?)
}
