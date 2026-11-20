// yack: brep-bench case swept/drum-end-screwed-through-rod
//! A rod threaded by an end mill's flat end.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let z = [0.0, 0.0, 1.0];
    let end = Frame::new([4.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0])?;
    let start = end.moved(&Rigid::rotation([0.0; 3], z, deg(-90.0))?).moved(&Rigid::translation([0.0, 0.0, -1.5]));
    let screw = Motion::screw([0.0; 3], z, rad(0.0), deg(1440.0), Lead::per_turn(6.0));
    cylinder(&Frame::WORLD, 5.0, 20.0)?.cut_swept(&cylinder(&start, 1.2, 8.0)?, &screw)
}
