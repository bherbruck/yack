// yack: brep-bench case boolean/indexed-pattern
//! A disk with six holes indexed around it.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let disk = cylinder(&Frame::at([0.0, 0.0, -5.0]), 20.0, 5.0)?;
    (0..6).try_fold(disk, |d, i| {
        let a = (60.0 * f64::from(i)).to_radians();
        d.difference(&cylinder(&Frame::at([12.0 * a.cos(), 12.0 * a.sin(), -6.0]), 2.0, 7.0)?)
    })
}
