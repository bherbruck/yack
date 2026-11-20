// yack: brep-bench case tangent/villarceau-half-torus
//! A torus cut by a plane tangent to it at two points: Villarceau's circles.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    // seam in the plane x = 0 on the side y > 0; keep 4z − 3x ≤ 0, the back of the plane's normal (−3, 0, 4)
    let ring = torus(&Frame::WORLD.turned(deg(90.0)), 5.0, 3.0)?;
    Ok(ring.split(&Plane::new([0.0, 0.0, 0.0], [-3.0, 0.0, 4.0])?)?.back)
}
