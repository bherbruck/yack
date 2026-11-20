//! Three rounds meeting at a cube's corner deleted, with the ball between them.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    let edges = [b.edge_at([10.0, 10.0, 5.0])?, b.edge_at([10.0, 5.0, 10.0])?, b.edge_at([5.0, 10.0, 10.0])?];
    let rounded = b.fillet(&edges, 2.0)?;
    rounded.delete_faces(&rounded.faces_where(|f| !matches!(f.kind(), yack::SurfaceKind::Plane { .. })))
}
