// yack: brep-bench case sweep/round-cornered-pipe
//! A round pipe turned around a sharp corner with a round joint.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let (x, y, z) = ([1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]);
    let first = cylinder(&Frame::new([-30.0, 0.0, 0.0], y, z, x)?, 3.0, 30.0)?;
    let second = cylinder(&Frame::new([0.0; 3], z, x, y)?, 3.0, 30.0)?;
    Solid::union_all(&[first, second, sphere(&Frame::WORLD, 3.0)?])
}
