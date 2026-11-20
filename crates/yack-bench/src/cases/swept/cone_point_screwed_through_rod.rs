// yack: brep-bench case swept/cone-point-screwed-through-rod
//! A rod threaded by a cone's point.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let z = [0.0, 0.0, 1.0];
    let base = Frame::new([12.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0], [-1.0, 0.0, 0.0])?;
    let start = base.moved(&Rigid::rotation([0.0; 3], z, deg(-90.0))?).moved(&Rigid::translation([0.0, 0.0, -1.5]));
    let bit = cone(&start, 8.0 * 30f64.to_radians().tan(), 0.0, 8.0)?;
    let screw = Motion::Screw { axis_point: [0.0; 3].into(), axis: z.into(), angle: deg(1440.0), rise_per_turn: 6.0 };
    cylinder(&Frame::WORLD, 5.0, 20.0)?.difference(&bit.swept(&screw)?)
}
