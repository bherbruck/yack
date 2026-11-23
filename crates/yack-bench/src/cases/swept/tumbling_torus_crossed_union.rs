// yack: brep-bench case swept/tumbling-torus-crossed-union
//! The union of opposing torus tumbles.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    // A and B: the torus rolled by t from -100° to 100°, 0.7·t about the upright line through (2, 0, 0), then −t (A) or
    // +t (B) about the leaning line through q along u, together; each sweep completed on its own, then combined
    let (l, a, h) = (deg(45.0), deg(20.0), 0.7071);
    let (q, u) = ([0.0, h * l.cos(), h * l.sin()], [a.cos(), a.sin() * l.cos(), a.sin() * l.sin()]);
    let roll = Motion::turn([2.0, 0.0, 0.0], Dir3::Z, deg(-70.0), deg(70.0));
    let lean = |s: f64| Motion::turn(q, u, deg(-100.0 * s), deg(100.0 * s));
    let ring = torus(&Frame::at([3.0, 0.0, 2.0]), 1.0, 0.22)?;
    let first = ring.swept(&Motion::together(vec![roll.clone(), lean(-1.0)]))?;
    first.union(&ring.swept(&Motion::together(vec![roll, lean(1.0)]))?)
}
