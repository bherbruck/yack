// yack: brep-bench case scale/micron-balls-united
//! Two balls a few microns across, united.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    sphere(&Frame::WORLD, 0.005)?.union(&sphere(&Frame::at([0.002, 0.003, 0.006]), 0.004)?)
}
