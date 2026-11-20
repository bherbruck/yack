// yack: brep-bench case thicken/cone-frustum
//! A conical sheet thickened along its normal, its rims cones.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let c = cone(&Frame::WORLD, 6.0, 3.0, 4.0)?;
    c.thicken(&[c.face_at([0.0, 4.5, 2.0])?], 1.0)
}
