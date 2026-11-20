// yack: brep-bench case geometry/hyperboloid
//! A hyperboloid of one sheet.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    // the branch x = 4 cosh t, z = 6 sinh t for |sinh t| ≤ 1 is the conic of weight cosh t = √2
    let r2 = 2f64.sqrt();
    let side = Loop::starting_at([0.0, -6.0]).line_to([4.0 * r2, -6.0]).conic_to([4.0 * r2, 6.0], [2.0 * r2, 0.0], r2);
    revolve(&Frame::WORLD, &Profile::new(side.line_to([0.0, 6.0]).close()?, vec![])?, deg(360.0))
}
