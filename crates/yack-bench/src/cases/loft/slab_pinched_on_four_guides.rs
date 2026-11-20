// yack: brep-bench case loft/slab-pinched-on-four-guides
//! A slab pinched thin on four Bézier guides.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let g = [[12.0, 3.2, 0.0], [18.0, -2.8, 8.0], [15.0, 0.2, 19.0], [3.0, 12.2, 24.0]];
    let signs = [[1.0, 1.0], [1.0, -1.0], [-1.0, 1.0], [-1.0, -1.0]];
    let guides = signs.iter().map(|&[sx, sy]| Spline3::bezier(&g.map(|[x, y, z]| [sx * x, sy * y, z])));
    let foot = (Frame::WORLD, Profile::rectangle([-12.0, -3.2], [12.0, 3.2])?);
    let top = (Frame::at([0.0, 0.0, 24.0]), Profile::rectangle([-3.0, -12.2], [3.0, 12.2])?);
    loft_guided(&[foot, top], &guides.collect::<Result<Vec<_>>>()?)
}
