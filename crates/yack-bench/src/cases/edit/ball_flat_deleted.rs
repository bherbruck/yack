// yack: brep-bench case edit/ball-flat-deleted
//! A ball's flat deleted: the ball is whole again.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let ball = sphere(&Frame::WORLD, 5.0)?.split(&Plane::new([0.0, 0.0, 3.0], [0.0, 0.0, -1.0])?)?.front;
    ball.delete_faces(&[ball.face_at([0.0, 0.0, 3.0])?])
}
