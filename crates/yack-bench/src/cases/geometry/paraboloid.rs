// yack: brep-bench case geometry/paraboloid
//! A paraboloid.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    // the parabola x² = 8z from its vertex to (8, 8) is the conic of weight 1 whose tangents meet at (4, 0)
    let side = Loop::starting_at([0.0, 0.0]).conic_to([8.0, 8.0], [4.0, 0.0], 1.0).line_to([0.0, 8.0]).close()?;
    revolve(&Frame::WORLD, &Profile::new(side, vec![])?, deg(360.0))
}
