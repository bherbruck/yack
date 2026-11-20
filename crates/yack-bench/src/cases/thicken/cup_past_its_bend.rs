// yack: brep-bench case thicken/cup-past-its-bend
//! A cup's skin thickened inward past its bend's radius.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let pen = Loop::starting_at([0.0, 0.0]).line_to([4.0, 0.0]).arc_ccw_to([6.0, 2.0], [4.0, 2.0]);
    let outline = pen.line_to([6.0, 10.0]).line_to([0.0, 10.0]).close()?;
    let cup = revolve(&Frame::WORLD, &Profile::new(outline, vec![])?, deg(360.0))?;
    let bend = cup.face_at([0.0, 4.0 + 2f64.sqrt(), 2.0 - 2f64.sqrt()])?;
    cup.thicken(&[cup.face_at([1.0, 1.0, 0.0])?, bend, cup.face_at([0.0, 6.0, 6.0])?], -3.0, Join::Round)
}
