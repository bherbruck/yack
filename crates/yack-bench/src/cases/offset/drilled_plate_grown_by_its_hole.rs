//! A drilled plate grown by its hole's radius.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let plate = block(&Frame::at([-10.0, -10.0, 0.0]), [20.0, 20.0, 4.0])?;
    plate.difference(&cylinder(&Frame::at([0.0, 0.0, -1.0]), 3.0, 6.0)?)?.offset(3.0)
}
