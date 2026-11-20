//! A tee of two cylinders with its junction rounded.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let bar = cylinder(&Frame::new([-20.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0])?, 6.0, 40.0)?;
    let tee = bar.union(&cylinder(&Frame::WORLD, 3.0, 15.0)?)?;
    tee.fillet(&[tee.edge_at([0.0, 3.0, 27f64.sqrt()])?], 1.0)
}
