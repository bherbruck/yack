// yack: brep-bench case offset/l-block-shrunk
//! An L-shaped plate shrunk by 1.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let l = Profile::polygon(&[[0.0, 0.0], [20.0, 0.0], [20.0, 5.0], [5.0, 5.0], [5.0, 15.0], [0.0, 15.0]])?;
    extrude(&Frame::WORLD, &l, 8.0)?.offset(-1.0)
}
