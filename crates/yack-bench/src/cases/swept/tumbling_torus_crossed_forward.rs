// yack: brep-bench case swept/tumbling-torus-crossed-forward
//! The forward member of a crossed pair of torus sweeps.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    // one roll t from -100.0° to 100.0°: 0.7·t about the upright line through (2, 0, 0), then -1.0·t about B
    let (l, a, h) = (45.0f64.to_radians(), 20.0f64.to_radians(), 0.7071);
    let q = Point3::new(0.0, h * l.cos(), h * l.sin());
    let u = Vec3::new(a.cos(), a.sin() * l.cos(), a.sin() * l.sin());
    let (p, z) = (Point3::new(2.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0));
    let roll = Motion::turn(p, z, deg(-70.0), deg(70.0));
    let tumble = Motion::turn(q, u, deg(100.0), deg(-100.0));
    torus(&Frame::at([3.0, 0.0, 2.0]), 1.0, 0.22)?.swept(&Motion::together(vec![roll, tumble]))
}
