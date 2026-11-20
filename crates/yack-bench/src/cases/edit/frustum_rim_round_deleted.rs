// yack: brep-bench case edit/frustum-rim-round-deleted
//! A cone frustum's rounded top rim deleted: the top and the cone meet again.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let frustum = cone(&Frame::WORLD, 6.0, 3.0, 6.0)?;
    let rounded = frustum.fillet(&[frustum.edge_at([0.0, 3.0, 6.0])?], 1.0)?;
    rounded.delete_faces(&rounded.faces_where(|f| matches!(f.kind(), yack::SurfaceKind::Torus))?)
}
