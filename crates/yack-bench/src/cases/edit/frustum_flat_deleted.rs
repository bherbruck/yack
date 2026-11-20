//! A cone frustum's flat deleted: the frustum is whole again.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let milled = cone(&Frame::WORLD, 6.0, 3.0, 20.0)?.split(&Plane::new([0.0, -4.0, 0.0], [0.0, 1.0, 0.0])?)?.front;
    milled.delete_faces(&[milled.face_at([0.0, -4.0, 5.0])?])
}
