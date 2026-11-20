//! A shaft's V-groove deleted: the shaft is whole again.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let groove = revolve(&Frame::WORLD, &Profile::polygon(&[[3.0, 10.0], [6.0, 7.0], [6.0, 13.0]])?, deg(360.0))?;
    let s = cylinder(&Frame::WORLD, 5.0, 20.0)?.difference(&groove)?;
    s.delete_faces(&[s.face_at([0.0, 4.0, 9.0])?, s.face_at([0.0, 4.0, 11.0])?])
}
