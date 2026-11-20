// yack: brep-bench case swept/key-carried-along-a-spline
//! A square key carried along a spline.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let plate = block(&Frame::at([-6.0, -6.0, -8.0]), [42.0, 32.0, 8.0])?;
    let path = Spline3::bezier(&[[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [20.0, 20.0, 0.0], [30.0, 20.0, 0.0]])?;
    let key = block(&Frame::at([-1.5, -0.8, -1.5]), [3.0, 1.6, 4.0])?;
    plate.cut_swept(&key, &Motion::along(path, FrameLaw::RotationMinimising))
}
