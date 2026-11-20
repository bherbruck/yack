//! A shaft's open keyway deleted: the shaft is whole again.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let s = cylinder(&Frame::WORLD, 5.0, 20.0)?;
    let s = s.difference(&block(&Frame::at([-1.0, -6.0, 8.0]), [2.0, 3.0, 13.0])?)?;
    let walls = [s.face_at([-1.0, -4.0, 15.0])?, s.face_at([1.0, -4.0, 15.0])?];
    s.delete_faces(&[s.face_at([0.0, -3.0, 15.0])?, walls[0], walls[1], s.face_at([0.0, -4.0, 8.0])?])
}
