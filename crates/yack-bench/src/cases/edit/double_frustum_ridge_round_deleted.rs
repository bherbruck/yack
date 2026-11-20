// yack: brep-bench case edit/double-frustum-ridge-round-deleted
//! Two cone frustums base to base, their rounded ridge deleted: the two cones meet again.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let part = cone(&Frame::WORLD, 3.0, 5.0, 4.0)?.union(&cone(&Frame::at([0.0, 0.0, 4.0]), 5.0, 2.0, 4.0)?)?;
    let rounded = part.fillet(&[part.edge_at([0.0, 5.0, 4.0])?], 1.0)?;
    rounded.delete_faces(&rounded.faces_where(|f| matches!(f.kind(), yack::SurfaceKind::Torus))?)
}
