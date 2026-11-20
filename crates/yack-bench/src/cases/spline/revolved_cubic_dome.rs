// yack: brep-bench case spline/revolved-cubic-dome
//! A dome turned from a cubic Bézier curve.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let ctrl = [[9.0, 0.0], [12.0, 4.0], [9.0, 10.0], [0.0, 10.0]].map(Point2::from).to_vec();
    let side = Segment::Nurbs { degree: 3, knots: vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0], ctrl, weights: None };
    let base = Segment::Line { a: [0.0, 0.0].into(), b: [9.0, 0.0].into() };
    let axis = Segment::Line { a: [0.0, 10.0].into(), b: [0.0, 0.0].into() };
    revolve(&Frame::WORLD, &Profile::new(Loop::new(vec![base, side, axis])?, vec![])?, deg(360.0))
}
