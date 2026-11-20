// yack: brep-bench case sweep/duct-elbow
//! A hollow square duct elbow.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let outer = Loop::polygon(&[[21.0, -9.0], [39.0, -9.0], [39.0, 9.0], [21.0, 9.0]])?;
    let hole = Loop::polygon(&[[23.0, -7.0], [23.0, 7.0], [37.0, 7.0], [37.0, -7.0]])?;
    revolve(&Frame::WORLD, &Profile::new(outer, vec![hole])?, deg(90.0))
}
