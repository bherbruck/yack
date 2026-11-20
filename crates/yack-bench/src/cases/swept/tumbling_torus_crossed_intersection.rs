// yack: brep-bench case swept/tumbling-torus-crossed-intersection
//! The intersection of opposing torus tumbles.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    // A and B: the torus rolled by t from -100° to 100°, 0.7·t about the upright line through (2, 0, 0), then −t (A) or
    // +t (B) about the leaning line through q along u, together; each sweep completed on its own, then combined
    let (l, a, h) = (45f64.to_radians(), 20f64.to_radians(), 0.7071);
    let q = Point3::new(0.0, h * l.cos(), h * l.sin());
    let u = Vec3::new(a.cos(), a.sin() * l.cos(), a.sin() * l.sin());
    let (p, z) = (Point3::new(2.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0));
    let roll = Motion::turn(p, z, deg(-70.0), deg(70.0));
    let lean = |s: f64| Motion::turn(q, u, deg(-100.0 * s), deg(100.0 * s));
    let ring = torus(&Frame::at([3.0, 0.0, 2.0]), 1.0, 0.22)?;
    let first = ring.swept(&Motion::together(vec![roll.clone(), lean(-1.0)]))?;
    first.intersection(&ring.swept(&Motion::together(vec![roll, lean(1.0)]))?)
}
