// yack: brep-bench case scale/micron-cylinders-crossed
//! Two cylinders a few microns across, crossed.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let first = cylinder(&Frame::along([-0.01, 0.0, 0.0], [1.0, 0.0, 0.0])?, 0.003, 0.02)?;
    first.intersection(&cylinder(&Frame::along([0.0, -0.01, 0.0], [0.0, 1.0, 0.0])?, 0.003, 0.02)?)
}
