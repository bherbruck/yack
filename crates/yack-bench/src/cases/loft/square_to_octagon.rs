// yack: brep-bench case loft/square-to-octagon
//! A square-to-octagon transition: corners lofted to sides.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let square = Profile::rectangle([-20.0, -20.0], [20.0, 20.0])?;
    let c = [[6., -14.], [14., -6.], [14., 6.], [6., 14.], [-6., 14.], [-14., 6.], [-14., -6.], [-6., -14.]];
    loft_through(&Frame::WORLD, &[(0.0, square), (30.0, Profile::polygon(&c)?)])
}
