// yack: brep-bench case thicken/plate-with-holes
//! A square sheet with two holes, thickened to both sides.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let outer = Loop::polygon(&[[-10.0, -10.0], [10.0, -10.0], [10.0, 10.0], [-10.0, 10.0]])?;
    let left = Loop::starting_at([-2.0, 0.0]).arc_cw_to([-2.0, 0.0], [-5.0, 0.0]).close()?;
    let right = Loop::starting_at([8.0, 0.0]).arc_cw_to([8.0, 0.0], [5.0, 0.0]).close()?;
    let plate = extrude(&Frame::at([0.0, 0.0, -1.0]), &Profile::new(outer, vec![left, right])?, 1.0)?;
    let sheet = plate.face_at([0.0, 5.0, 0.0])?;
    plate.thicken(&[sheet], 1.0, Join::Round)?.union(&plate.thicken(&[sheet], -1.0, Join::Round)?)
}
