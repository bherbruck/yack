// yack: brep-bench case edit/rounded-frustum-rounds-deleted
//! A cone frustum's two rounded rims deleted: its ends and its cone meet again.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let frustum = cone(&Frame::WORLD, 6.0, 3.0, 6.0)?;
    let rounded = frustum.fillet(&[frustum.edge_at([0.0, 3.0, 6.0])?, frustum.edge_at([0.0, 6.0, 0.0])?], 1.5)?;
    rounded.delete_faces(&rounded.faces_where(|f| matches!(f.kind(), yack::SurfaceKind::Torus)))
}
