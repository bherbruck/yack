// yack: brep-bench case edit/shaft-ring-groove-deleted
//! A shaft's ring groove deleted: the shaft is whole again.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let ring = cylinder(&Frame::at([0.0, 0.0, 8.0]), 6.0, 4.0)?;
    let ring = ring.difference(&cylinder(&Frame::at([0.0, 0.0, 7.0]), 4.0, 6.0)?)?;
    let s = cylinder(&Frame::WORLD, 5.0, 20.0)?.difference(&ring)?;
    s.delete_faces(&[s.face_at([0.0, 4.0, 10.0])?, s.face_at([0.0, 4.5, 8.0])?, s.face_at([0.0, 4.5, 12.0])?])
}
