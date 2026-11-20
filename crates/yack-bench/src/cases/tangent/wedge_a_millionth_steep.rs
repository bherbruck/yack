// yack: brep-bench case tangent/wedge-a-millionth-steep
//! A wedge a millionth steep.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let cube = block(&Frame::at([-5.0, -5.0, -10.0]), [10.0, 10.0, 10.0])?;
    Ok(cube.split(&Plane::new([0.0, 0.0, 0.0], [0.0, -1e-6, 1.0])?)?.front)
}
