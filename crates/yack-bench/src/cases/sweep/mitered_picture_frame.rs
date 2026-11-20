// yack: brep-bench case sweep/mitered-picture-frame
//! A picture frame: a rabbeted molding mitered around a rectangle.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    // the section in (s, z): s outward from the path, so the frame's x points away from the rectangle's centre
    let molding = Profile::polygon(&[[0.0, 2.0], [3.0, 2.0], [3.0, 0.0], [8.0, 0.0], [8.0, 6.0], [0.0, 6.0]])?;
    let path = [[30.0, -20.0, 0.0], [-30.0, -20.0, 0.0], [-30.0, 20.0, 0.0], [30.0, 20.0, 0.0]].map(Point3::from);
    let start = Frame::new([30.0, -20.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0], [-1.0, 0.0, 0.0])?;
    sweep_mitered(&start, &molding, &path, true)
}
