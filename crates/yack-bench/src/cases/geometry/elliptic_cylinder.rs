// yack: brep-bench case geometry/elliptic-cylinder
//! An elliptic cylinder.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    // four quarter ellipses, each the conic with weight cos 45° on its bounding box corner
    let w = 0.5f64.sqrt();
    let half = Loop::starting_at([5.0, 0.0]).conic_to([0.0, 3.0], [5.0, 3.0], w).conic_to([-5.0, 0.0], [-5.0, 3.0], w);
    let ellipse = half.conic_to([0.0, -3.0], [-5.0, -3.0], w).conic_to([5.0, 0.0], [5.0, -3.0], w).close()?;
    extrude(&Frame::WORLD, &Profile::new(ellipse, vec![])?, 10.0)
}
