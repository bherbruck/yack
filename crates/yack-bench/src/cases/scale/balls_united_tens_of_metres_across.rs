// yack: brep-bench case scale/balls-united-tens-of-metres-across
//! Two balls tens of metres across, united.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    sphere(&Frame::WORLD, 50000.0)?.union(&sphere(&Frame::at([20000.0, 30000.0, 60000.0]), 40000.0)?)
}
