// yack: brep-bench case fillet/nurbs-cylinder-rim
//! A NURBS cylinder with its top rim rounded.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let c = [[1.0, 0.0], [1.0, 1.0], [0.0, 1.0], [-1.0, 1.0], [-1.0, 0.0], [-1.0, -1.0], [0.0, -1.0], [1.0, -1.0]];
    let ctrl = c.iter().chain(c.first()).map(|&[x, y]| Point2::from([5.0 * x, 5.0 * y])).collect();
    let weights = Some((0..9).map(|i| if i % 2 == 0 { 1.0 } else { 0.5f64.sqrt() }).collect());
    let knots = vec![0.0, 0.0, 0.0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1.0, 1.0, 1.0];
    let circle = Loop::new(vec![Segment::Nurbs { degree: 2, knots, ctrl, weights }])?;
    let s = extrude(&Frame::WORLD, &Profile::new(circle, vec![])?, 10.0)?;
    s.fillet(&[s.edge_at([0.0, 5.0, 10.0])?], 1.0)
}
