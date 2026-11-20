// yack: brep-bench case swept/two-beads-slid-through-rod
//! A rod grooved by a wheel of two beads.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let (x, y, z) = ([1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]);
    let first = torus(&Frame::new([11.5, 0.0, -9.0], z, x, y)?, 6.0, 2.0)?;
    let wheel = first.union(&torus(&Frame::new([11.5, 1.5, -9.0], z, x, y)?, 6.0, 1.5)?)?;
    cylinder(&Frame::WORLD, 5.0, 20.0)?.cut_swept(&wheel, &Motion::slide([0.0, 0.0, 40.0]))
}
