// yack: brep-bench case geometry/periodic-bspline-ring
//! A ring turned from a periodic B-spline.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let ctrl = [[13.0, 3.0], [7.0, 3.0], [7.0, -3.0], [13.0, -3.0]].map(Point2::from).to_vec();
    let section = Loop::new(vec![Segment::PeriodicSpline { degree: 2, knots: (0..9).map(f64::from).collect(), ctrl }])?;
    revolve(&Frame::WORLD, &Profile::new(section, vec![])?, deg(360.0))
}
