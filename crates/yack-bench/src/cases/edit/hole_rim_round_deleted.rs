//! A drilled block's rounded hole rim deleted: the top and the bore meet again.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let b = block(&Frame::WORLD, [20.0, 10.0, 10.0])?.difference(&cylinder(&Frame::at([10.0, 5.0, -1.0]), 2.0, 12.0)?)?;
    let rounded = b.fillet(&[b.edge_at([10.0, 7.0, 10.0])?], 1.0)?;
    rounded.delete_faces(&rounded.faces_where(|f| matches!(f.kind(), yack::SurfaceKind::Torus)))
}
