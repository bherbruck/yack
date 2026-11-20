// yack: brep-bench case parts/flange
//! A bored mounting flange with six bolt holes.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let body = cylinder(&Frame::WORLD, 32.0, 6.0)?.union(&cylinder(&Frame::at([0.0, 0.0, 6.0]), 15.0, 10.0)?)?;
    let body = body.difference(&cylinder(&Frame::at([0.0, 0.0, -1.0]), 8.0, 18.0)?)?;
    (0..6).try_fold(body, |b, i| {
        let a = (60.0 * f64::from(i)).to_radians();
        b.difference(&cylinder(&Frame::at([24.0 * a.cos(), 24.0 * a.sin(), -1.0]), 3.0, 8.0)?)
    })
}
