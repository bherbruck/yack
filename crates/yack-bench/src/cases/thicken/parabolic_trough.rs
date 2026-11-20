// yack: brep-bench case thicken/parabolic-trough
//! A parabolic trough, a B-spline sheet, thickened on its concave side.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let side = Frame::new([0.0, 10.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0])?;
    let outline = Loop::starting_at([-4.0, 2.0]).conic_to([4.0, 2.0], [0.0, -2.0], 1.0).line_to([0.0, 6.0]).close()?;
    let trough = extrude(&side, &Profile::new(outline, vec![])?, 10.0)?;
    trough.thicken(&[trough.face_at([0.0, 5.0, 0.0])?], -1.0)
}
