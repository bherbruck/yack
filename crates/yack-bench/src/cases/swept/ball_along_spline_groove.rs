// yack: brep-bench case swept/ball-along-spline-groove
//! A groove carried along a spline by a ball.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let plate = block(&Frame::at([-6.0, -6.0, -8.0]), [42.0, 32.0, 8.0])?;
    let path = Spline3::bezier(&[[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [20.0, 20.0, 0.0], [30.0, 20.0, 0.0]])?;
    plate.cut_swept(&sphere(&Frame::WORLD, 3.0)?, &Motion::along(path, FrameLaw::Fixed))
}
