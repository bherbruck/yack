//! A ball pierced off center.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let axis = Frame::new([5.0, -21.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])?;
    sphere(&Frame::WORLD, 20.0)?.difference(&cylinder(&axis, 6.0, 42.0)?)
}
