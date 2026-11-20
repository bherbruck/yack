// yack: brep-bench case sweep/mitered-elbow
//! A square bar mitered around a sharp corner.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let square = Profile::rectangle([-5.0, -5.0], [5.0, 5.0])?;
    let path = [[0.0, 0.0, 0.0], [40.0, 0.0, 0.0], [20.0, 20.0 * 3f64.sqrt(), 0.0]].map(Point3::from);
    sweep_mitered(&Frame::along([0.0, 0.0, 0.0], [1.0, 0.0, 0.0])?, &square, &path, false)
}
