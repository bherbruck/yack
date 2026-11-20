// yack: brep-bench case spline/pipe-on-twisted-cubic
//! A pipe along a twisted cubic Bézier curve.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let (x, y, z) = ([0.0, 0.0, 1.0], [0.8, -0.6, 0.0], [0.6, 0.8, 0.0]);
    let ctrl = [[0.0, 0.0, 0.0], [12.0, 16.0, 0.0], [26.0, 8.0, 0.0], [24.0, 0.0, 16.0]];
    let local = ctrl.map(|p: [f64; 3]| [x, y, z].map(|a| a[0] * p[0] + a[1] * p[1] + a[2] * p[2]));
    let disk = Profile::circle([0.0, 0.0], 8.0)?;
    sweep_path(&Frame::new([0.0; 3], x, y, z)?, &disk, &Spline3::bezier(&local)?, deg(0.0))
}
