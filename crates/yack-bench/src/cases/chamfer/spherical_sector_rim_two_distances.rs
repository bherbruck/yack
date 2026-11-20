// yack: brep-bench case chamfer/spherical-sector-rim-two-distances
//! A spherical sector whose rim is chamfered with two distances.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let s = sphere(&Frame::WORLD, 5.0)?.intersection(&cone(&Frame::WORLD, 0.0, 8.0, 6.0)?)?;
    s.chamfer_two(s.edge_at([8f64.sqrt(), 8f64.sqrt(), 3.0])?, s.face_at([0.0, 2.4, 1.8])?, 2.0, 1.0)
}
