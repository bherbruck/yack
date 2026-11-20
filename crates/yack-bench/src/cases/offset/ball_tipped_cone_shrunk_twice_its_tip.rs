// yack: brep-bench case offset/ball-tipped-cone-shrunk-twice-its-tip
//! A ball-tipped cone shrunk by twice its tip's radius.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let (s5, c) = (5f64.sqrt(), 12.0 - 5f64.sqrt());
    let side = Loop::starting_at([0.0, 0.0]).line_to([6.0, 0.0]).line_to([2.0 / s5, 12.0 - 4.0 / s5]);
    let outline = side.arc_ccw_to([0.0, c + 1.0], [0.0, c]).close()?;
    revolve(&Frame::WORLD, &Profile::new(outline, vec![])?, deg(360.0))?.offset(-2.0)
}
