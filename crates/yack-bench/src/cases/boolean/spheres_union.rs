// yack: brep-bench case boolean/spheres-union
//! Two overlapping balls, united.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    sphere(&Frame::WORLD, 5.0)?.union(&sphere(&Frame::at([6.0, 0.0, 0.0]), 5.0)?)
}
