// yack: brep-bench case parts/pocketed-tray
//! A pocketed tray with four blind-bored standoffs.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let tray = block(&Frame::at([-36.0, 0.0, -24.0]), [72.0, 16.0, 48.0])?;
    let tray = tray.difference(&block(&Frame::at([-33.0, 0.0, -21.0]), [66.0, 13.0, 42.0])?)?;
    [[26.0, 14.0], [-26.0, 14.0], [-26.0, -14.0], [26.0, -14.0]].iter().try_fold(tray, |t, &[x, z]| {
        let f = Frame::new([x, 5.0, z], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])?;
        t.union(&cylinder(&f, 5.0, 8.0)?.difference(&cylinder(&f, 2.0, 8.0)?)?)
    })
}
