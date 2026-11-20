//! Two crossed cylinders, their common part.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let x = Frame::new([-15.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0])?;
    let y = Frame::new([0.0, -15.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])?;
    cylinder(&x, 5.0, 30.0)?.intersection(&cylinder(&y, 5.0, 30.0)?)
}
