// yack: brep-bench case edit/shaft-round-groove-deleted
//! A shaft's round-bottomed groove deleted: the shaft is whole again.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let s = cylinder(&Frame::WORLD, 5.0, 20.0)?.difference(&torus(&Frame::at([0.0, 0.0, 10.0]), 5.0, 1.5)?)?;
    s.delete_faces(&s.faces_where(|f| matches!(f.kind(), yack::SurfaceKind::Torus)))
}
