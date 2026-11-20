// yack: brep-bench case swept/drum-screwed-across-ring-sector
//! A helical groove milled by a drum screwed across a block.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let z = [0.0, 0.0, 1.0];
    let sector = revolve(&Frame::WORLD, &Profile::rectangle([8.0, -1.0], [12.0, 1.0])?, deg(60.0))?;
    let start = Frame::at([10.0, 0.0, -0.5 - 0.5 * 140.0 / 360.0]).moved(&Rigid::rotation([0.0; 3], z, deg(-140.0))?);
    let screw = Motion::Screw { axis_point: [0.0; 3].into(), axis: z.into(), angle: deg(280.0), rise_per_turn: 0.5 };
    sector.difference(&cylinder(&start, 1.0, 1.0)?.swept(&screw)?)
}
