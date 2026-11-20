// yack: brep-bench case thicken/bent-flange-past-its-bend
//! A bent sheet thickened inside its bend past the bend's radius.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let side = Frame::new([0.0, 8.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0])?;
    let pen = Loop::starting_at([-10.0, 0.0]).line_to([0.0, 0.0]).arc_ccw_to([2.0, 2.0], [0.0, 2.0]);
    let s = extrude(&side, &Profile::new(pen.line_to([2.0, 12.0]).line_to([-10.0, 12.0]).close()?, vec![])?, 8.0)?;
    let r = 2f64.sqrt();
    s.thicken(&s.faces_at(&[[-5.0, 4.0, 0.0], [r, 4.0, 2.0 - r], [2.0, 4.0, 7.0]])?, -3.0, Join::Round)
}
