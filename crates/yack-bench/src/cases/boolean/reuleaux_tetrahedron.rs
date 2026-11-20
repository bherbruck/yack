//! A Reuleaux tetrahedron: four balls in common.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let (r3, h) = (3f64.sqrt(), 10.0 * (2.0f64 / 3.0).sqrt());
    let corners = [[10.0, 0.0, 0.0], [5.0, 5.0 * r3, 0.0], [5.0, 5.0 * r3 / 3.0, h]];
    corners.iter().try_fold(sphere(&Frame::WORLD, 10.0)?, |s, &c| s.intersection(&sphere(&Frame::at(c), 10.0)?))
}
