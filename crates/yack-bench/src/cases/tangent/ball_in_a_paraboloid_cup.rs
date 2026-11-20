// yack: brep-bench case tangent/ball-in-a-paraboloid-cup
//! A ball in a paraboloid cup that fits its bottom.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    // the parabola x² = 10z to (√50, 5) is the conic of weight 1 whose end tangents meet at (√50/2, 0)
    let r = 50f64.sqrt();
    let side = Loop::starting_at([0.0, 0.0]).conic_to([r, 5.0], [r / 2.0, 0.0], 1.0).line_to([0.0, 5.0]).close()?;
    let cup = revolve(&Frame::WORLD, &Profile::new(side, vec![])?, deg(360.0))?;
    cup.union(&sphere(&Frame::at([0.0, 0.0, 5.0]), 5.0)?)
}
