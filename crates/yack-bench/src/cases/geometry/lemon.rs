// yack: brep-bench case geometry/lemon
//! A lemon: a circular segment turned about its chord.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let segment = Loop::starting_at([0.0, -4.0]).arc_ccw_to([0.0, 4.0], [-3.0, 0.0]).close()?;
    revolve(&Frame::WORLD, &Profile::new(segment, vec![])?, deg(360.0))
}
