// yack: brep-bench case swept/ring-carried-along-a-spline
//! A wheel carried along a spline, cutting with its rim.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let plate = block(&Frame::at([-6.0, -6.0, -8.0]), [42.0, 32.0, 8.0])?;
    let path = Spline3::bezier(&[[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [20.0, 20.0, 0.0], [30.0, 20.0, 0.0]])?;
    let wheel = torus(&Frame::new([0.0, 0.0, 0.5], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])?, 1.5, 0.5)?;
    plate.difference(&wheel.swept(&Motion::Along { path, law: FrameLaw::RotationMinimising })?)
}
