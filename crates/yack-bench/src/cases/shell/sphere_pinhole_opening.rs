// yack: brep-bench case shell/sphere-pinhole-opening
//! A ball cut by a plane and shelled open there, the cavity barely reaching it.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let cut = sphere(&Frame::WORLD, 5.0)?.split(&Plane::new([0.0, 0.0, 3.9999], [0.0, 0.0, -1.0])?)?.front;
    cut.shell(&[cut.face_at([0.5, 0.5, 3.9999])?], 1.0)
}
