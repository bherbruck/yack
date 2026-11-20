// yack: brep-bench case swept/worm-wheel-hobbed-by-its-worm
//! A worm wheel hobbed by its worm.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    // data: the thread's section at θ = 0 (r from 7.5 to 12.5, |ξ| ≤ π/2 − (r − 10)·tan 20°), screwed at lead 2π
    // (rise 1 per radian) along the worm's axis from x = −10π (whole turns before 0) to past x = 20; cut to |x| ≤ 20
    let (tau, tan20) = (std::f64::consts::TAU, 20f64.to_radians().tan());
    let w = |r: f64| std::f64::consts::FRAC_PI_2 - (r - 10.0) * tan20;
    let section = Profile::polygon(&[[7.5, -w(7.5)], [12.5, -w(12.5)], [12.5, w(12.5)], [7.5, w(7.5)]])?;
    let (y, z, x) = ([0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]);
    let start = Frame::new([-5.0 * tau, 40.0, 0.0], y, z, x)?;
    let thread = screw(&start, &section, Lead::per_radian(1.0), rad(9.0 * tau))?;
    let worm = thread.intersection(&cylinder(&Frame::new([-20.0, 40.0, 0.0], y, z, x)?, 13.0, 40.0)?)?;
    // seen from the wheel: turned by t about the worm's axis, then by −t/30 about z, t from 0 to 60π
    let p = Point3::new(0.0, 40.0, 0.0);
    let spin = Motion::turn(p, x, rad(0.0), rad(30.0 * tau));
    let mesh = Motion::turn(Point3::ORIGIN, z, deg(0.0), deg(-360.0));
    cylinder(&Frame::at([0.0, 0.0, -6.0]), 32.0, 12.0)?.cut_swept(&worm, &Motion::together(vec![spin, mesh]))
}
