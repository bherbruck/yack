// yack: brep-bench case thicken/half-pipe-outward
//! A half-pipe sheet thickened away from its axis.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let half = cylinder(&Frame::WORLD, 5.0, 10.0)?.split(&Plane::new([0.0, 0.0, 0.0], [0.0, 1.0, 0.0])?)?.front;
    half.thicken(&[half.face_at([0.0, 5.0, 5.0])?], 1.0)
}
