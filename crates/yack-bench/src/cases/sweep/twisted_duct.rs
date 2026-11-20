// yack: brep-bench case sweep/twisted-duct
//! A square duct turned to a diamond.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let a = [[10., 0., -10.], [-10., 0., -10.], [-10., 0., 10.], [10., 0., 10.]];
    let b = [[0., 40., -12.], [-12., 40., 0.], [0., 40., 12.], [12., 40., 0.]];
    let guides = a.iter().zip(b).map(|(&p, q)| Spline3::bezier(&[p, q])).collect::<Result<Vec<_>>>()?;
    let foot = Frame::new([0.0; 3], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])?;
    let end = Frame::new([0.0, 40.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])?;
    let diamond = Profile::polygon(&[[12.0, 0.0], [0.0, 12.0], [-12.0, 0.0], [0.0, -12.0]])?;
    loft_guided(&[(foot, Profile::rectangle([-10.0, -10.0], [10.0, 10.0])?), (end, diamond)], &guides)
}
