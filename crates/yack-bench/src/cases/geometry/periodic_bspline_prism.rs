// yack: brep-bench case geometry/periodic-bspline-prism
//! A prism on a periodic B-spline.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let ctrl = [[10.0, 10.0], [-10.0, 10.0], [-10.0, -10.0], [10.0, -10.0]].map(Point2::from).to_vec();
    let section = Loop::new(vec![Segment::PeriodicSpline { degree: 2, knots: (0..9).map(f64::from).collect(), ctrl }])?;
    extrude(&Frame::WORLD, &Profile::new(section, vec![])?, 10.0)
}
