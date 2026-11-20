// yack: brep-bench case edit/hole-both-rim-rounds-deleted
//! A drilled block's two rounded hole rims deleted: the faces and the bore meet again at both ends.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let b = block(&Frame::WORLD, [20.0, 10.0, 10.0])?.difference(&cylinder(&Frame::at([10.0, 5.0, -1.0]), 2.0, 12.0)?)?;
    let rounded = b.fillet(&[b.edge_at([10.0, 7.0, 10.0])?, b.edge_at([10.0, 7.0, 0.0])?], 1.0)?;
    rounded.delete_faces(&rounded.faces_where(|f| matches!(f.kind(), yack::SurfaceKind::Torus))?)
}
