// yack: brep-bench case sweep/hollow-reducer
//! A hollow square reducer.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let along_y = Frame::new([0.0; 3], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0])?;
    let (outside, hole) = (Profile::rectangle([-12.; 2], [12.; 2])?, Profile::rectangle([-10.; 2], [10.; 2])?);
    let hollow = loft(&along_y, &hole, 0.4, 40.0)?;
    loft(&along_y, &outside, 0.5, 40.0)?.difference(&hollow)
}
