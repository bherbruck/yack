// yack: brep-bench case swept/spiroid-face-gear-hobbed-by-its-worm
//! A spiroid face gear hobbed by its tapered worm.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    // data: the thread's section where it starts, a whole turn before x = 0 (pitch radius R = 6 − 2π·tan 5°): r from
    // R − 2.5 to R + 2.5, |ξ| ≤ π/2 − (r − R)·tan 20°; screwed at lead 2π, widening tan 5° per radian
    let (tau, tan5, tan20) = (std::f64::consts::TAU, 5f64.to_radians().tan(), 20f64.to_radians().tan());
    let (r0, w) = (6.0 - tau * tan5, |u: f64| std::f64::consts::FRAC_PI_2 - u * tan20);
    let (a, b) = (r0 - 2.5, r0 + 2.5);
    let section = Profile::polygon(&[[a, -w(-2.5)], [b, -w(2.5)], [b, w(2.5)], [a, w(-2.5)]])?;
    let (y, z, x) = ([0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]);
    let start = Frame::new([-tau, 30.0, 6.0], y, z, x)?;
    let thread = screw_tapered(&start, &section, Lead::per_radian(1.0), rad(10.0 * tau), tan5)?;
    let worm = thread.intersection(&cylinder(&Frame::new([0.0, 30.0, 6.0], y, z, x)?, 14.0, 52.0)?)?;
    let quad = Profile::polygon(&[[40.0, -9.0], [52.0, -9.0], [52.0, -1.7], [40.0, -0.3]])?;
    let ring = revolve(&Frame::WORLD, &quad, deg(360.0))?;
    // seen from the gear: turned by t about the worm's axis, then by −t/30 about z, t from 0 to 60π
    let p = Point3::new(0.0, 30.0, 6.0);
    let spin = Motion::turn(p, x, rad(0.0), rad(30.0 * tau));
    let mesh = Motion::turn(Point3::ORIGIN, z, deg(0.0), deg(-360.0));
    ring.cut_swept(&worm, &Motion::together(vec![spin, mesh]))
}
