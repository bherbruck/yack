// yack: brep-bench case swept/torus-slid-through-rod
//! A rod grooved by a wheel slid along it.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let wheel = torus(&Frame::new([11.5, 0.0, -9.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])?, 6.0, 2.0)?;
    cylinder(&Frame::WORLD, 5.0, 20.0)?.difference(&wheel.swept(&Motion::Slide { by: [0.0, 0.0, 38.0].into() })?)
}
