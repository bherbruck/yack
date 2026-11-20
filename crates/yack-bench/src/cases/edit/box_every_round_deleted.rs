//! A box with every edge rounded, all twenty rounds deleted.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let b = block(&Frame::WORLD, [20.0, 16.0, 12.0])?;
    let rounded = b.fillet(&b.edges(), 2.0)?;
    rounded.delete_faces(&rounded.faces_where(|f| !matches!(f.kind(), yack::SurfaceKind::Plane { .. })))
}
