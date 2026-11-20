// yack: brep-bench case swept/drum-pushed-through-plate
//! A slot punched by a drum pushed edge first.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let side = Frame::new([6.0, 0.0, -5.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0])?;
    let push = Motion::slide([0.0, 0.0, 16.0]);
    block(&Frame::at([0.0, -5.0, 0.0]), [20.0, 10.0, 4.0])?.cut_swept(&cylinder(&side, 2.0, 8.0)?, &push)
}
