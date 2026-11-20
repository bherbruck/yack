// yack: brep-bench case swept/ring-pushed-in-past-its-fold
//! A ring pushed into a block at so steep a lean its inner outline folds.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let (s, c) = (65f64.to_radians().sin(), 65f64.to_radians().cos());
    let ring = torus(&Frame::new([0.0, 0.0, 4.0], [c, 0.0, -s], [0.0, 1.0, 0.0], [s, 0.0, c])?, 3.0, 1.0)?;
    let hole = ring.swept(&Motion::slide([0.0, 0.0, -8.0]))?;
    block(&Frame::at([-8.0, -8.0, -8.0]), [16.0, 16.0, 8.0])?.difference(&hole)
}
