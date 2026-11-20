// yack: brep-bench case boolean/cross-drilled-cube
//! A cube drilled through on a grid three ways.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let (x, y, z, c) = ([1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [7.5, 15.0, 22.5]);
    let across = c.iter().flat_map(|&a| [Frame::new([-1.0, a, 15.0], y, z, x), Frame::new([a, -1.0, 15.0], z, x, y)]);
    let frames = c.iter().flat_map(|&a| c.map(|b| Ok(Frame::at([a, b, -1.0])))).chain(across);
    let holes = frames.map(|f| cylinder(&f?, 2.5, 32.0)).collect::<Result<Vec<_>>>()?;
    block(&Frame::WORLD, [30.0, 30.0, 30.0])?.difference_all(&holes)
}
