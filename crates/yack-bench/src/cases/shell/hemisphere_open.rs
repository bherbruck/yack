// yack: brep-bench case shell/hemisphere-open
//! A half ball shelled open at its flat face: a bowl.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let bowl = sphere(&Frame::WORLD, 5.0)?.split(&Plane::new([0.0, 0.0, 0.0], [0.0, 0.0, -1.0])?)?.front;
    bowl.shell(&[bowl.face_at([1.0, 1.0, 0.0])?], 1.0)
}
