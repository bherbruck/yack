// yack: brep-bench case swept/core-drill-pushed-aslant-through-plate
//! A core drill pushed along its leaning axis through a plate.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let (s, c) = (30f64.to_radians().sin(), 30f64.to_radians().cos());
    let foot = Frame::new([-18.0 * s, 0.0, 2.0 - 18.0 * c], [c, 0.0, -s], [0.0, 1.0, 0.0], [s, 0.0, c])?;
    let drill = cylinder(&foot, 2.5, 12.0)?.difference(&cylinder(&foot, 1.5, 12.0)?)?;
    let plate = block(&Frame::at([-10.0, -10.0, 0.0]), [20.0, 20.0, 4.0])?;
    plate.difference(&drill.swept(&Motion::Slide { by: [24.0 * s, 0.0, 24.0 * c].into() })?)
}
