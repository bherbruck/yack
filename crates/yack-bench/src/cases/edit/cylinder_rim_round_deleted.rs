// yack: brep-bench case edit/cylinder-rim-round-deleted
//! A cylinder's rounded rim deleted: the top and the side meet again.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let c = cylinder(&Frame::WORLD, 6.0, 10.0)?;
    let rounded = c.fillet(&[c.edge_at([0.0, 6.0, 10.0])?], 2.0)?;
    rounded.delete_faces(&rounded.faces_where(|f| matches!(f.kind(), yack::SurfaceKind::Torus))?)
}
