// yack: brep-bench case edit/domed-post-round-deleted
//! A post capped by a ball, its rounded rim deleted: the cylinder and the ball meet again.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let dome = sphere(&Frame::at([0.0, 0.0, 3.0]), 5.0)?.split(&Plane::new([0.0, 0.0, 6.0], [0.0, 0.0, 1.0])?)?.front;
    let post = cylinder(&Frame::WORLD, 4.0, 6.0)?.union(&dome)?;
    let rounded = post.fillet(&[post.edge_at([8f64.sqrt(), 8f64.sqrt(), 6.0])?], 1.0)?;
    rounded.delete_faces(&rounded.faces_where(|f| matches!(f.kind(), yack::SurfaceKind::Torus)))
}
