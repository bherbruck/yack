// yack: brep-bench case edit/block-edge-round-deleted
//! A block's rounded edge deleted: the top and the side meet again.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let b = block(&Frame::WORLD, [20.0, 10.0, 10.0])?;
    let rounded = b.fillet(&[b.edge_at([20.0, 5.0, 10.0])?], 2.0)?;
    rounded.delete_faces(&rounded.faces_where(|f| matches!(f.kind(), yack::SurfaceKind::Cylinder { .. })))
}
