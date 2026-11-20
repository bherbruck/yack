// yack: brep-bench case swept/v-wheel-screwed-through-rod
//! A rod threaded by a V wheel.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let (z, h, t) = ([0.0, 0.0, 1.0], (2.0 * std::f64::consts::PI * 4.5 / 30.0).atan(), 2.5 * 30f64.to_radians().tan());
    let hub = Frame::new([14.0, 0.0, 0.0], [-1.0, 0.0, 0.0], [0.0, h.sin(), h.cos()], [0.0, h.cos(), -h.sin()])?;
    let start = hub.moved(&Rigid::rotation([0.0; 3], z, deg(-144.0))?).moved(&Rigid::translation([0.0, 0.0, -12.0]));
    let wheel = revolve(&start, &Profile::polygon(&[[7.5, -t], [10.0, 0.0], [7.5, t]])?, deg(360.0))?;
    let screw = Motion::screw([0.0; 3], z, rad(0.0), deg(528.0), Lead::per_turn(30.0));
    cylinder(&Frame::WORLD, 5.0, 20.0)?.cut_swept(&wheel, &screw)
}
