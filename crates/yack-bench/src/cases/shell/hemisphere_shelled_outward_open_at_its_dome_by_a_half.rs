// yack: brep-bench case shell/hemisphere-shelled-outward-open-at-its-dome-by-a-half
//! A half ball shelled outward by a half, open at its dome.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let half = sphere(&Frame::WORLD, 5.0)?.split(&Plane::new([0.0, 0.0, 0.0], [0.0, 0.0, 1.0])?)?.front;
    half.shell_outward(&[half.face_at([0.0, 0.0, 5.0])?], 0.5)
}
