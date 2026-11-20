// yack: brep-bench case swept/bull-nose-carried-along-a-spline
//! A leaning bull-nose mill carried along a spline.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let (s, c) = (30f64.to_radians().sin(), 30f64.to_radians().cos());
    let lean = Frame::new([0.0, 0.0, -1.0], [c, 0.0, -s], [0.0, 1.0, 0.0], [s, 0.0, c])?;
    let mill = cylinder(&lean, 2.0, 3.0)?;
    let mill = mill.fillet(&[mill.edge_at([0.0, 2.0, -1.0])?], 0.5)?;
    let plate = block(&Frame::at([-6.0, -6.0, -8.0]), [42.0, 32.0, 8.0])?;
    let path = Spline3::bezier(&[[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [20.0, 20.0, 0.0], [30.0, 20.0, 0.0]])?;
    plate.cut_swept(&mill, &Motion::along(path, FrameLaw::RotationMinimising))
}
