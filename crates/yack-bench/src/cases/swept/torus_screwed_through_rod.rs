// yack: brep-bench case swept/torus-screwed-through-rod
//! A rod grooved by a wheel screwed along it.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let (z, h) = ([0.0, 0.0, 1.0], (2.0 * std::f64::consts::PI * 5.5 / 30.0).atan());
    let set = Frame::new([11.5, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, -h.sin(), -h.cos()], [0.0, h.cos(), -h.sin()])?;
    let start = set.moved(&Rigid::rotation([0.0; 3], z, deg(-108.0))?).moved(&Rigid::translation([0.0, 0.0, -9.0]));
    let screw = Motion::screw([0.0; 3], z, rad(0.0), deg(456.0), Lead::per_turn(30.0));
    cylinder(&Frame::WORLD, 5.0, 20.0)?.cut_swept(&torus(&start, 6.0, 2.0)?, &screw)
}
