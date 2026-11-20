// yack: brep-bench case edit/turned-post-cone-turned-rounds-deleted
//! A turned post's rounded shoulder deleted, its cone made turned a radian about the axis: the
//! cylinder and the cone meet again.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let (c, s) = (1f64.cos(), 1f64.sin());
    let top = cone(&Frame::new([0.0, 0.0, 6.0], [c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0])?, 5.0, 2.0, 4.0)?;
    let post = cylinder(&Frame::WORLD, 5.0, 6.0)?.union(&top)?;
    let shoulder = post.edges_between(post.face_at([0.0, 5.0, 3.0])?, post.face_at([0.0, 3.5, 8.0])?)?;
    let rounded = post.fillet(&shoulder, 1.0)?;
    rounded.delete_faces(&rounded.faces_where(|f| matches!(f.kind(), yack::SurfaceKind::Torus))?)
}
