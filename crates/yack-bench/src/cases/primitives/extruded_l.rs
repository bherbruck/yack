// yack: brep-bench case primitives/extruded-l
//! An L-shaped plate.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let l = Profile::polygon(&[[0.0, 0.0], [20.0, 0.0], [20.0, 5.0], [5.0, 5.0], [5.0, 15.0], [0.0, 15.0]])?;
    extrude(&Frame::WORLD, &l, 8.0)
}
