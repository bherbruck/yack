// yack: brep-bench case boolean/drilled-plate
//! A plate drilled nine times.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let plate = block(&Frame::WORLD, [100.0, 20.0, 10.0])?;
    (1..10).try_fold(plate, |p, i| p.difference(&cylinder(&Frame::at([10.0 * f64::from(i), 10.0, -1.0]), 2.0, 12.0)?))
}
