// yack: brep-bench case shell/ball-tipped-cone-hollowed-past-its-tip
//! A ball-tipped cone hollowed to a wall thicker than its tip's radius.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let (s, c) = (5f64.sqrt(), 12.0 - 5f64.sqrt());
    let outline = Loop::starting_at([0.0, 0.0]).line_to([6.0, 0.0]).line_to([2.0 / s, 12.0 - 4.0 / s]);
    let outline = outline.arc_ccw_to([0.0, c + 1.0], [0.0, c]).close()?;
    let tipped = revolve(&Frame::WORLD, &Profile::new(outline, vec![])?, deg(360.0))?;
    tipped.shell(&[], 1.5)
}
