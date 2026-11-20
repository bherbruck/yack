// yack: brep-bench case thicken/pan-thickened-three-times-its-bend
//! A pan with a tight bend thickened inward by three times the bend's radius.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let pen = Loop::starting_at([0.0, 0.0]).line_to([5.0, 0.0]).arc_ccw_to([6.0, 1.0], [5.0, 1.0]);
    let outline = pen.line_to([6.0, 8.0]).line_to([0.0, 8.0]).close()?;
    let pan = revolve(&Frame::WORLD, &Profile::new(outline, vec![])?, deg(360.0))?;
    let bend = pan.face_at([0.0, 5.0 + 0.5f64.sqrt(), 1.0 - 0.5f64.sqrt()])?;
    pan.thicken(&[pan.face_at([1.0, 1.0, 0.0])?, bend, pan.face_at([0.0, 6.0, 5.0])?], -3.0)
}
