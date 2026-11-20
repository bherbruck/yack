// yack: brep-bench case edit/block-two-rounds-deleted
//! A block's two rounded top edges deleted: the top and both sides meet again.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let b = block(&Frame::WORLD, [20.0, 10.0, 10.0])?;
    let rounded = b.fillet(&[b.edge_at([10.0, 0.0, 10.0])?, b.edge_at([10.0, 10.0, 10.0])?], 2.0)?;
    rounded.delete_faces(&rounded.faces_where(|f| matches!(f.kind(), yack::SurfaceKind::Cylinder { .. })))
}
