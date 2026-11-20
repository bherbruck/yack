// yack: brep-bench case shell/capsule-open-at-its-dome
//! A capsule's end shelled open at its dome: the walls run on past their face to meet it.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let outline = Loop::starting_at([0.0, 0.0]).line_to([5.0, 0.0]).line_to([5.0, 10.0]);
    let outline = outline.arc_ccw_to([0.0, 15.0], [0.0, 10.0]).close()?;
    let capsule = revolve(&Frame::WORLD, &Profile::new(outline, vec![])?, deg(360.0))?;
    capsule.shell(&[capsule.face_at([0.0, -3.0, 14.0])?], 1.0)
}
