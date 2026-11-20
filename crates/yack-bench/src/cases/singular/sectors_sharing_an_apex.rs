// yack: brep-bench case singular/sectors-sharing-an-apex
//! Two spherical sectors sharing an apex.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let a = 10.0 / 2f64.sqrt();
    let outline = Loop::starting_at([0.0, 0.0]).line_to([a, a]).arc_ccw_to([0.0, 10.0], [0.0, 0.0]).close()?;
    let (sector, turn) = (Profile::new(outline, vec![])?, Rigid::rotation([0.0; 3], [0.0, 1.0, 0.0], deg(60.0))?);
    let first = revolve(&Frame::WORLD, &sector, deg(360.0))?;
    first.intersection(&revolve(&Frame::WORLD.moved(&turn), &sector, deg(360.0))?)
}
