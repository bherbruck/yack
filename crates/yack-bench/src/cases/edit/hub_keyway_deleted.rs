//! A hub's keyway deleted: the bore is round again.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let tube = cylinder(&Frame::WORLD, 10.0, 20.0)?;
    let tube = tube.difference(&cylinder(&Frame::at([0.0, 0.0, -1.0]), 4.0, 22.0)?)?;
    let hub = tube.difference(&block(&Frame::at([-1.0, -5.0, -1.0]), [2.0, 5.0, 22.0])?)?;
    let walls = [hub.face_at([-1.0, -4.5, 10.0])?, hub.face_at([1.0, -4.5, 10.0])?];
    hub.delete_faces(&[hub.face_at([0.0, -5.0, 10.0])?, walls[0], walls[1]])
}
