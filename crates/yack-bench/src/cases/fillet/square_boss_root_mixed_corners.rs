//! A square boss rounded at its corners and its root, two concave rounds and one convex at each
//! corner.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let s = block(&Frame::WORLD, [40.0, 30.0, 5.0])?.union(&block(&Frame::at([12.0, 7.0, 5.0]), [16.0, 16.0, 10.0])?)?;
    let upright = [[12.0, 7.0, 10.0], [28.0, 7.0, 10.0], [28.0, 23.0, 10.0], [12.0, 23.0, 10.0]];
    let s = s.fillet(&upright.iter().map(|&p| s.edge_at(p)).collect::<Result<Vec<_>>>()?, 3.0)?;
    s.fillet(&[s.edge_at([20.0, 7.0, 5.0])?], 2.0)
}
