// yack: brep-bench case tangent/midsphere-common
//! A cube trimmed by the ball tangent to its edges.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let cube = block(&Frame::at([-5.0, -5.0, -5.0]), [10.0, 10.0, 10.0])?;
    cube.intersection(&sphere(&Frame::WORLD, 5.0 * 2f64.sqrt())?)
}
