// yack: brep-bench case parts/turned-shaft
//! A turned shaft with a rounded shoulder.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let pen = Loop::starting_at([0.0, 0.0]).line_to([20.0, 0.0]).line_to([20.0, 60.0]).line_to([16.0, 60.0]);
    let outline = pen.arc_cw_to([12.0, 64.0], [16.0, 64.0]).line_to([12.0, 160.0]).line_to([0.0, 160.0]).close()?;
    revolve(&Frame::WORLD, &Profile::new(outline, vec![])?, deg(360.0))
}
