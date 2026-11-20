// yack: brep-bench case shell/sphere-opening-sealed
//! A ball cut by a plane and shelled open there, the cavity never reaching it.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let cut = sphere(&Frame::WORLD, 5.0)?.split(&Plane::new([0.0, 0.0, 4.5], [0.0, 0.0, -1.0])?)?.front;
    cut.shell_rounded(&[cut.face_at([0.5, 0.5, 4.5])?], 1.0)
}
