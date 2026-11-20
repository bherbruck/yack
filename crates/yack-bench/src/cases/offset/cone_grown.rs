// yack: brep-bench case offset/cone-grown
//! A cone grown by 1.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    cone(&Frame::WORLD, 3.0, 0.0, 4.0)?.offset(1.0)
}
