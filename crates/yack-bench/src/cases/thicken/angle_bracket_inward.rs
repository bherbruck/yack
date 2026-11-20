// yack: brep-bench case thicken/angle-bracket-inward
//! A creased sheet thickened inside its crease, the moved faces crossing.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let b = block(&Frame::WORLD, [10.0, 8.0, 10.0])?;
    b.thicken(&[b.face_at([5.0, 4.0, 0.0])?, b.face_at([0.0, 4.0, 5.0])?], -1.0, Join::Round)
}
