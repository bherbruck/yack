// yack: brep-bench case thicken/rectangle
//! A rectangular sheet thickened by 2.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let b = block(&Frame::at([0.0, 0.0, -1.0]), [10.0, 8.0, 1.0])?;
    b.thicken(&[b.face_at([5.0, 4.0, 0.0])?], 2.0, Join::Round)
}
