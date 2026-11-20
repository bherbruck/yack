// yack: brep-bench case spline/square-on-planar-cubic
//! A square bar bent along a cubic Bézier curve.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let (x, y, z) = ([0.0, 0.0, 1.0], [0.6, -0.8, 0.0], [0.8, 0.6, 0.0]);
    let ctrl = [[0.0, 0.0, 0.0], [16.0, 12.0, 0.0], [18.0, 26.0, 0.0], [12.0, 34.0, 0.0]];
    let local = ctrl.map(|p: [f64; 3]| [x, y, z].map(|a| a[0] * p[0] + a[1] * p[1] + a[2] * p[2]));
    let square = Profile::rectangle([-10.0, -10.0], [10.0, 10.0])?;
    sweep_path(&Frame::new([0.0; 3], x, y, z)?, &square, &Spline3::bezier(&local)?, deg(0.0))
}
