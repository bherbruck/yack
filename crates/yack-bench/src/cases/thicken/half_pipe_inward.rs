// yack: brep-bench case thicken/half-pipe-inward
//! A half-pipe sheet thickened toward its axis.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let half = cylinder(&Frame::WORLD, 5.0, 10.0)?.split(&Plane::new([0.0, 0.0, 0.0], [0.0, 1.0, 0.0])?)?.front;
    half.thicken(&[half.face_at([0.0, 5.0, 5.0])?], -2.0)
}
