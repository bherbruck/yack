//! A shaft's two opposite flats deleted: the shaft is whole again.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let s = cylinder(&Frame::WORLD, 5.0, 20.0)?.split(&Plane::new([4.0, 0.0, 0.0], [-1.0, 0.0, 0.0])?)?.front;
    let s = s.split(&Plane::new([-4.0, 0.0, 0.0], [1.0, 0.0, 0.0])?)?.front;
    s.delete_faces(&[s.face_at([4.0, 0.0, 10.0])?, s.face_at([-4.0, 0.0, 10.0])?])
}
