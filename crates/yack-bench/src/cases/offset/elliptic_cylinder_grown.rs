// yack: brep-bench case offset/elliptic-cylinder-grown
//! An elliptic cylinder grown by 1.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let w = 0.5f64.sqrt();
    let half = Loop::starting_at([5.0, 0.0]).conic_to([0.0, 3.0], [5.0, 3.0], w).conic_to([-5.0, 0.0], [-5.0, 3.0], w);
    let ellipse = half.conic_to([0.0, -3.0], [-5.0, -3.0], w).conic_to([5.0, 0.0], [5.0, -3.0], w).close()?;
    extrude(&Frame::WORLD, &Profile::new(ellipse, vec![])?, 10.0)?.offset(1.0)
}
