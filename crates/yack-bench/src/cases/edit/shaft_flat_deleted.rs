// yack: brep-bench case edit/shaft-flat-deleted
//! A flat milled along a shaft deleted: the shaft is whole again.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let shaft = cylinder(&Frame::WORLD, 5.0, 20.0)?.split(&Plane::new([0.0, -4.0, 0.0], [0.0, 1.0, 0.0])?)?.front;
    shaft.delete_faces(&[shaft.face_at([0.0, -4.0, 10.0])?])
}
