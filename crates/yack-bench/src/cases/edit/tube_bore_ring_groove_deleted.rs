// yack: brep-bench case edit/tube-bore-ring-groove-deleted
//! A tube's internal ring groove deleted: the bore is whole again.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let tube = cylinder(&Frame::WORLD, 10.0, 20.0)?.difference(&cylinder(&Frame::at([0.0, 0.0, -1.0]), 4.0, 22.0)?)?;
    let t = tube.difference(&cylinder(&Frame::at([0.0, 0.0, 8.0]), 6.0, 4.0)?)?;
    t.delete_faces(&[t.face_at([0.0, 6.0, 10.0])?, t.face_at([0.0, 5.0, 8.0])?, t.face_at([0.0, 5.0, 12.0])?])
}
