// yack: brep-bench case swept/tumbling-torus-near-orthogonal
//! A thin torus tumbled about nearly perpendicular axes.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    // one roll t from -180.0° to 180.0°: 1.01·t about the upright line through (2, 0, 0), then -1.0·t about B
    let (l, a, h) = (1.0f64.to_radians(), 89.0f64.to_radians(), 0.0);
    let q = Point3::new(0.0, h * l.cos(), h * l.sin());
    let u = Vec3::new(a.cos(), a.sin() * l.cos(), a.sin() * l.sin());
    let (p, z) = (Point3::new(2.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0));
    let roll = Motion::turn(p, z, deg(-181.8), deg(181.8));
    let tumble = Motion::turn(q, u, deg(180.0), deg(-180.0));
    torus(&Frame::at([2.02, 0.0, 1.0]), 1.0, 0.08)?.swept(&Motion::together(vec![roll, tumble]))
}
