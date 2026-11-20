//! An L-block's rounded inside corner deleted: the corner is sharp again.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let l = Profile::polygon(&[[0.0, 0.0], [20.0, 0.0], [20.0, 5.0], [5.0, 5.0], [5.0, 15.0], [0.0, 15.0]])?;
    let b = extrude(&Frame::WORLD, &l, 8.0)?;
    let rounded = b.fillet(&[b.edge_at([5.0, 5.0, 4.0])?], 2.0)?;
    rounded.delete_faces(&rounded.faces_where(|f| matches!(f.kind(), yack::SurfaceKind::Cylinder { .. })))
}
