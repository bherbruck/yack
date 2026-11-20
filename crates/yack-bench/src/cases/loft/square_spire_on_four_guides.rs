// yack: brep-bench case loft/square-spire-on-four-guides
//! A square spire on four Bézier ridges.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    // each ridge: the cubic Bézier (9, 9, 0), (15, 15, 8), (12, 12, 19), (0, 0, 24), mirrored to each corner
    let ridge = [[9.0, 0.0], [15.0, 8.0], [12.0, 19.0], [0.0, 24.0]];
    let corners = [(1.0, 1.0), (-1.0, 1.0), (-1.0, -1.0), (1.0, -1.0)];
    let ridges = corners.map(|(x, y)| Spline3::bezier(&ridge.map(|[r, z]| [r * x, r * y, z])));
    let foot = Profile::rectangle([-9.0, -9.0], [9.0, 9.0])?;
    loft_to_point(&Frame::WORLD, &foot, [0.0, 0.0, 24.0], &ridges.into_iter().collect::<Result<Vec<_>>>()?)
}
