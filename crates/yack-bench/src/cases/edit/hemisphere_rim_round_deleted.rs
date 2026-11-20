// yack: brep-bench case edit/hemisphere-rim-round-deleted
//! A hemisphere's rounded rim deleted: the flat and the ball meet again.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let half = sphere(&Frame::WORLD, 5.0)?.split(&Plane::new([0.0, 0.0, 0.0], [0.0, 0.0, 1.0])?)?.front;
    let rounded = half.fillet(&[half.edge_at([0.0, 5.0, 0.0])?], 1.0)?;
    rounded.delete_faces(&rounded.faces_where(|f| matches!(f.kind(), yack::SurfaceKind::Torus)))
}
