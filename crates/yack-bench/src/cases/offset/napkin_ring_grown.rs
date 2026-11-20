//! A napkin ring grown by 1.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let ball = sphere(&Frame::WORLD, 5.0)?;
    ball.difference(&cylinder(&Frame::at([0.0, 0.0, -6.0]), 3.0, 12.0)?)?.offset(1.0)
}
