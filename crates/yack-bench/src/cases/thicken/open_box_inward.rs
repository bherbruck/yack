// yack: brep-bench case thicken/open-box-inward
//! An open box's sheet thickened inward.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    let pts = [[5.0, 5.0, 0.0], [0.0, 5.0, 5.0], [10.0, 5.0, 5.0], [5.0, 0.0, 5.0], [5.0, 10.0, 5.0]];
    b.thicken(&pts.map(|p| b.face_at(p)).into_iter().collect::<Result<Vec<_>>>()?, -1.0)
}
