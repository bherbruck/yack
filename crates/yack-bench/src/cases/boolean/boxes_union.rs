// yack: brep-bench case boolean/boxes-union
//! Two overlapping blocks, united.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let a = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    a.union(&block(&Frame::at([5.0, 5.0, 0.0]), [10.0, 10.0, 10.0])?)
}
