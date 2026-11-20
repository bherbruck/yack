// yack: brep-bench case loft/square-to-octagon
//! A square-to-octagon transition: corners lofted to sides.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    // every wall is flat: four trapezoids (square side to parallel octagon side) and four triangles (square
    // corner to cut side), so the hood is the block between z = 0 and 30 cut by the eight walls' planes
    let walls = [(1.0, 0.0, 5.0), (0.0, 1.0, 5.0), (-1.0, 0.0, 5.0), (0.0, -1.0, 5.0)];
    let corners = [(1.0, 1.0, 1.5), (-1.0, 1.0, 1.5), (-1.0, -1.0, 1.5), (1.0, -1.0, 1.5)];
    let hood = block(&Frame::at([-20.0, -20.0, 0.0]), [40.0, 40.0, 30.0])?;
    walls.iter().chain(&corners).try_fold(hood, |h, &(x, y, k)| {
        Ok(h.split(&Plane::new([20.0 * x, 20.0 * y, 0.0], [k * x, k * y, 1.0])?)?.back)
    })
}
