// yack: brep-bench case thicken/sphere-skin
//! A closed spherical sheet thickened into a hollow ball.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let ball = sphere(&Frame::WORLD, 5.0)?;
    ball.thicken(&[ball.face_at([0.0, 3.0, 4.0])?], 1.0)
}
