// yack: brep-bench case swept/tumbling-torus-thin
//! A thin torus tumbled through space.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    // one roll t from -75.0° to 75.0°: 0.25·t about the upright line through (2, 0, 0), then -1.0·t about B
    let (l, a, h) = (45.0f64.to_radians(), 0.0f64.to_radians(), 0.5f64.sqrt());
    let q = Point3::new(0.0, h * l.cos(), h * l.sin());
    let u = Vec3::new(a.cos(), a.sin() * l.cos(), a.sin() * l.sin());
    let (p, z) = (Point3::new(2.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0));
    let roll = Motion::turn(p, z, deg(-18.75), deg(18.75));
    let tumble = Motion::turn(q, u, deg(75.0), deg(-75.0));
    torus(&Frame::at([3.0, 0.0, 2.0]), 1.0, 0.015)?.swept(&Motion::together(vec![roll, tumble]))
}
