// yack: brep-bench case edit/frustum-ring-groove-deleted
//! A cone frustum's ring groove deleted: the frustum is whole again.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let ring = cylinder(&Frame::at([0.0, 0.0, 8.0]), 7.0, 4.0)?;
    let ring = ring.difference(&cylinder(&Frame::at([0.0, 0.0, 7.0]), 4.0, 6.0)?)?;
    let s = cone(&Frame::WORLD, 6.0, 3.0, 20.0)?.difference(&ring)?;
    s.delete_faces(&[s.face_at([0.0, 4.0, 10.0])?, s.face_at([0.0, 4.4, 8.0])?, s.face_at([0.0, 4.1, 12.0])?])
}
