// yack: brep-bench case tangent/paraboloid-tip-on-face
//! A paraboloid's tip on a block's face.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    // the parabola x² = 8z from its vertex to (10, 12.5) is the conic of weight 1 whose end tangents meet at (5, 0)
    let side = Loop::starting_at([0.0, 0.0]).conic_to([10.0, 12.5], [5.0, 0.0], 1.0).line_to([0.0, 12.5]).close()?;
    let paraboloid = revolve(&Frame::WORLD, &Profile::new(side, vec![])?, deg(360.0))?;
    block(&Frame::at([-10.0, -10.0, 0.0]), [20.0, 20.0, 10.0])?.union(&paraboloid)
}
