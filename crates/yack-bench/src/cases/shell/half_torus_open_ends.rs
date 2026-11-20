//! A half torus shelled open at both ends: a curved pipe.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let half = torus(&Frame::WORLD, 6.0, 2.0)?.split(&Plane::new([0.0, 0.0, 0.0], [0.0, 1.0, 0.0])?)?.front;
    half.shell(&[half.face_at([6.0, 0.0, 0.0])?, half.face_at([-6.0, 0.0, 0.0])?], 0.5)
}
