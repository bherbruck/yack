// yack: brep-bench case shell/sphere-open-at-a-cap
//! A ball cut by a plane and shelled open there: the rim meets the walls aslant.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let cut = sphere(&Frame::WORLD, 5.0)?.split(&Plane::new([0.0, 0.0, 3.0], [0.0, 0.0, -1.0])?)?.front;
    cut.shell(&[cut.face_at([1.0, 1.0, 3.0])?], 1.0)
}
