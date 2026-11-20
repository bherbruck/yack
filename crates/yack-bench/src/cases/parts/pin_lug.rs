// yack: brep-bench case parts/pin-lug
//! A pin lug.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let along_y = Frame::new([60.0, -6.0, 20.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])?;
    block(&Frame::at([0.0, -5.0, 0.0]), [80.0, 10.0, 40.0])?.difference(&cylinder(&along_y, 8.0, 12.0)?)
}
