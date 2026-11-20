// yack: brep-bench case sweep/hollow-reducer
//! A hollow square reducer.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let along_y = Frame::new([0.0; 3], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0])?;
    let (outside, top) = (Profile::rectangle([-12., -12.], [12., 12.])?, Profile::rectangle([-6., -6.], [6., 6.])?);
    let (hole, hole_top) = (Profile::rectangle([-10., -10.], [10., 10.])?, Profile::rectangle([-4., -4.], [4., 4.])?);
    let hollow = loft(&along_y, &hole, 0.4, 40.0, &hole_top)?;
    loft(&along_y, &outside, 0.5, 40.0, &top)?.difference(&hollow)
}
