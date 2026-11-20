// yack: brep-bench case swept/drafted-slot-screwed-through-rod
//! A rod threaded by a drafted bar of slotted section.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    // the slot in the plane x = 12, drawn lowered 1.5 (where the screw starts, at its turn of −90°), carried 8 along −x
    let slot = Loop::starting_at([-1.5, -1.0]).line_to([1.5, -1.0]).arc_ccw_to([1.5, 1.0], [1.5, 0.0]);
    let slot = slot.line_to([-1.5, 1.0]).arc_ccw_to([-1.5, -1.0], [-1.5, 0.0]).close()?;
    let at = Frame::new([12.0, 0.0, -1.5], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0], [-1.0, 0.0, 0.0])?;
    let bar = extrude_drafted(&at, &Profile::new(slot, vec![])?, 8.0, deg(3.0), false)?;
    let screw = Motion::screw([0.0; 3], [0.0, 0.0, 1.0], deg(-90.0), deg(1350.0), Lead::per_turn(6.0));
    cylinder(&Frame::WORLD, 5.0, 20.0)?.cut_swept(&bar, &screw)
}
