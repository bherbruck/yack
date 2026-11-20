// yack: brep-bench case edit/rounded-box-top-rim-chain-deleted
//! A rounded box's rounded top rim deleted: the top and the walls meet again all round.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let b = block(&Frame::WORLD, [20.0, 12.0, 8.0])?;
    let sides = [b.edge_at([0.0, 0.0, 4.0])?, b.edge_at([20.0, 0.0, 4.0])?, b.edge_at([20.0, 12.0, 4.0])?];
    let b = b.fillet(&[sides[0], sides[1], sides[2], b.edge_at([0.0, 12.0, 4.0])?], 3.0)?;
    let rimmed = b.fillet(&b.edges_of(b.face_at([10.0, 6.0, 8.0])?)?, 1.0)?;
    let chain = rimmed.faces_where(|f| matches!(f.kind(), yack::SurfaceKind::Cylinder { radius, .. } if radius < 2.0)
        || matches!(f.kind(), yack::SurfaceKind::Torus))?;
    rimmed.delete_faces(&chain)
}
