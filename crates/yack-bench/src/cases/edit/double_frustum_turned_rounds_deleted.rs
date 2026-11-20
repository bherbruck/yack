//! Two cone frustums base to base, the upper made turned two radians about the axis, their rounded
//! ridge deleted: the two cones meet again.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let (c, s) = (2f64.cos(), 2f64.sin());
    let upper = cone(&Frame::new([0.0, 0.0, 4.0], [c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0])?, 5.0, 2.0, 4.0)?;
    let part = cone(&Frame::WORLD, 3.0, 5.0, 4.0)?.union(&upper)?;
    let ridge = part.edges_between(part.face_at([0.0, 4.0, 2.0])?, part.face_at([0.0, 3.5, 6.0])?)?;
    let rounded = part.fillet(&ridge, 1.0)?;
    rounded.delete_faces(&rounded.faces_where(|f| matches!(f.kind(), yack::SurfaceKind::Torus)))
}
