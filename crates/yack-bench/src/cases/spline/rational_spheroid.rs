// yack: brep-bench case spline/rational-spheroid
//! A spheroid turned from a weighted quadratic B-spline.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let ctrl = [[0.0, -10.0], [6.0, -10.0], [6.0, 0.0], [6.0, 10.0], [0.0, 10.0]].map(Point2::from).to_vec();
    let (h, knots) = (0.5f64.sqrt(), vec![0.0, 0.0, 0.0, 0.5, 0.5, 1.0, 1.0, 1.0]);
    let side = Segment::Nurbs { degree: 2, knots, ctrl, weights: Some(vec![1.0, h, 1.0, h, 1.0]) };
    let axis = Segment::Line { a: [0.0, 10.0].into(), b: [0.0, -10.0].into() };
    revolve(&Frame::WORLD, &Profile::new(Loop::new(vec![side, axis])?, vec![])?, deg(360.0))
}
