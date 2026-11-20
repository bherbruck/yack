// yack: brep-bench case swept/square-thread-at-a-growing-lead
//! A rod threaded at a lead that grows along it.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    // rise 6s + 0.3s² at the turn θ (s = θ/360°), written in φ = θ − f, the angle turned from `from` = f = −90°
    let (a, b, f) = (6.0 / std::f64::consts::TAU, 0.3 / std::f64::consts::TAU.powi(2), -std::f64::consts::FRAC_PI_2);
    let lead = Lead::polynomial(vec![a * f + b * f * f, a + 2.0 * b * f, b]);
    let z: Vec3 = Vec3::new(0.0, 0.0, 1.0);
    let climb = Motion::screw(Point3::ORIGIN, z, deg(-90.0), deg(1260.0), lead);
    let bar = block(&Frame::at([4.0, -8.0, -1.0]), [8.0, 16.0, 2.0])?;
    cylinder(&Frame::WORLD, 5.0, 20.0)?.cut_swept(&bar, &climb)
}
