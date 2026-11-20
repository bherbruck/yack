// yack: brep-bench case sweep/mitered-pipe-on-cube-edges
//! A round pipe mitered along three edges of a cube.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let path = [[0.0, 0.0, 0.0], [30.0, 0.0, 0.0], [30.0, 30.0, 0.0], [30.0, 30.0, 30.0]].map(Point3::from);
    sweep_mitered(&Frame::along([0.0, 0.0, 0.0], [1.0, 0.0, 0.0])?, &Profile::circle([0.0, 0.0], 2.0)?, &path, false)
}
