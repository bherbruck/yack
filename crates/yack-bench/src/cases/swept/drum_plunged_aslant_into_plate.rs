// yack: brep-bench case swept/drum-plunged-aslant-into-plate
//! A drum plunged aslant into a plate, stopping inside.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let side = Frame::new([6.0, 0.0, -5.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0])?;
    let push = Motion::Slide { by: [8.0 * 0.3, 0.0, 8.0 * 0.91f64.sqrt()].into() };
    block(&Frame::at([0.0, -5.0, 0.0]), [20.0, 10.0, 4.0])?.difference(&cylinder(&side, 2.0, 8.0)?.swept(&push)?)
}
