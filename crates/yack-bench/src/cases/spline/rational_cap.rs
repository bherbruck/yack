// yack: brep-bench case spline/rational-cap
//! A ball's cap turned from a weighted quadratic arc.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let r3 = 3f64.sqrt();
    let pen = Loop::starting_at([0.0, -5.0]).line_to([5.0 * r3, -5.0]);
    let outline = pen.conic_to([0.0, 10.0], [10.0 * r3, 10.0], 0.5).close()?;
    revolve(&Frame::WORLD, &Profile::new(outline, vec![])?, deg(360.0))
}
