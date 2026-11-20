// yack: brep-bench case geometry/elliptic-hole
//! A block with an elliptic hole.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let w = 0.5f64.sqrt();
    let half = Loop::starting_at([6.0, 0.0]).conic_to([0.0, 3.0], [6.0, 3.0], w).conic_to([-6.0, 0.0], [-6.0, 3.0], w);
    let ellipse = half.conic_to([0.0, -3.0], [-6.0, -3.0], w).conic_to([6.0, 0.0], [6.0, -3.0], w).close()?;
    let hole = extrude(&Frame::at([0.0, 0.0, -1.0]), &Profile::new(ellipse, vec![])?, 10.0)?;
    block(&Frame::at([-12.0, -8.0, 0.0]), [24.0, 16.0, 8.0])?.difference(&hole)
}
