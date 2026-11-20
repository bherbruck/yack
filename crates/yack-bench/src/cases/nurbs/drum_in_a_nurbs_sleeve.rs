// yack: brep-bench case nurbs/drum-in-a-nurbs-sleeve
//! A drum united with a sleeve whose bore is a NURBS cylinder.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let c = [[0., 5.], [5., 5.], [5., 0.], [5., -5.], [0., -5.], [-5., -5.], [-5., 0.], [-5., 5.], [0., 5.]];
    let weights = Some((0..9).map(|i| if i % 2 == 0 { 1.0 } else { 0.5f64.sqrt() }).collect());
    let knots = vec![0.0, 0.0, 0.0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1.0, 1.0, 1.0];
    let bore = Loop::new(vec![Segment::Nurbs { degree: 2, knots, ctrl: c.map(Point2::from).to_vec(), weights }])?;
    let rim = Profile::circle([0.0, 0.0], 7.0)?.outer().clone();
    let sleeve = extrude(&Frame::at([0.0, 0.0, 2.0]), &Profile::new(rim, vec![bore])?, 6.0)?;
    cylinder(&Frame::WORLD, 5.0, 10.0)?.union(&sleeve)
}
