// yack: brep-bench case parts/shelf-bracket
//! A shelf bracket with a rounded inside corner.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let pen = Loop::starting_at([0.0, 0.0]).line_to([20.0, 0.0]).line_to([20.0, 92.0]);
    let pen = pen.arc_cw_to([28.0, 100.0], [28.0, 92.0]).line_to([100.0, 100.0]).line_to([100.0, 120.0]);
    let outline = pen.line_to([0.0, 120.0]).close()?;
    let xz = Frame::new([0.0, 5.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0])?;
    extrude(&xz, &Profile::new(outline, vec![])?, 10.0)
}
