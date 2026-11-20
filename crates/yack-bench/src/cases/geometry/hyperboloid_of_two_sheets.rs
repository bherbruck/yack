// yack: brep-bench case geometry/hyperboloid-of-two-sheets
//! A cap of a hyperboloid of two sheets.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    // the branch x = 4 sinh t, z = 3 cosh t from t = 0 to cosh t = 2 is the conic of weight cosh(t/2) = √1.5
    let (r3, w) = (3f64.sqrt(), 1.5f64.sqrt());
    let side = Loop::starting_at([0.0, 3.0]).conic_to([4.0 * r3, 6.0], [4.0 / r3, 3.0], w).line_to([0.0, 6.0]).close()?;
    revolve(&Frame::WORLD, &Profile::new(side, vec![])?, deg(360.0))
}
