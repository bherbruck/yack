// yack: brep-bench case spline/peanut-prism
//! A peanut with a thin waist, a periodic spline extruded.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let c = [[14., 0.], [12., 8.], [4., 6.], [0., -2.625], [-4., 6.], [-12., 8.], [-14., 0.], [-12., -8.], [-4., -6.]];
    let c = [c.as_slice(), &[[0., 2.625], [4., -6.], [12., -8.]]].concat();
    // the twelve points of a period, then the first three again (one convention: n + degree points)
    let ctrl = c.iter().chain(c.iter().take(3)).map(|&p| Point2::from(p)).collect();
    let curve = Segment::PeriodicSpline { degree: 3, knots: (0..19).map(f64::from).collect(), ctrl };
    extrude(&Frame::WORLD, &Profile::new(Loop::new(vec![curve])?, vec![])?, 10.0)
}
