// yack: brep-bench case thicken/torus-inner-half
//! The inner half of a torus thickened toward its axis, a saddle.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let inner = torus(&Frame::WORLD, 6.0, 2.0)?.intersection(&cylinder(&Frame::at([0.0, 0.0, -3.0]), 6.0, 6.0)?)?;
    inner.thicken(&[inner.face_at([0.0, 4.0, 0.0])?], 1.0)
}
