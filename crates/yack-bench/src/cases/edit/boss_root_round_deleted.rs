// yack: brep-bench case edit/boss-root-round-deleted
//! A boss's rounded root deleted: the boss stands on the plate with a sharp foot.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let part = cylinder(&Frame::at([0.0, 0.0, -4.0]), 8.0, 4.0)?.union(&cylinder(&Frame::WORLD, 3.0, 8.0)?)?;
    let rounded = part.fillet(&[part.edge_at([0.0, 3.0, 0.0])?], 1.0)?;
    rounded.delete_faces(&rounded.faces_where(|f| matches!(f.kind(), yack::SurfaceKind::Torus)))
}
