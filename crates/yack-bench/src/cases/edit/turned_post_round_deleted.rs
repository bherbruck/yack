// yack: brep-bench case edit/turned-post-round-deleted
//! A turned post's rounded shoulder deleted: its cylinder and its cone meet again.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let post = cylinder(&Frame::WORLD, 5.0, 6.0)?.union(&cone(&Frame::at([0.0, 0.0, 6.0]), 5.0, 2.0, 4.0)?)?;
    let rounded = post.fillet(&[post.edge_at([0.0, 5.0, 6.0])?], 1.0)?;
    rounded.delete_faces(&rounded.faces_where(|f| matches!(f.kind(), yack::SurfaceKind::Torus)))
}
