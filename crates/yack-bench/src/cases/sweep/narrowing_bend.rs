// yack: brep-bench case sweep/narrowing-bend
//! A hollow square duct narrowing around a bend.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    // outside and hole each narrow evenly (9 to 6 and 7 to 4 half-widths), so the wall stays 2 thick
    let start = Frame::new([30.0, 0.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0])?;
    let bend = [PathPiece::Arc { radius: 30.0, angle: deg(90.0) }];
    let (outer, hole) = (Profile::rectangle([-9.0, -9.0], [9.0, 9.0])?, Profile::rectangle([-7.0, -7.0], [7.0, 7.0])?);
    let outside = sweep_scaled(&start, &outer, &bend, deg(0.0), 6.0 / 9.0)?;
    outside.difference(&sweep_scaled(&start, &hole, &bend, deg(0.0), 4.0 / 7.0)?)
}
