// yack: brep-bench case thicken/tube
//! A cylindrical sheet thickened into a tube.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let c = cylinder(&Frame::WORLD, 5.0, 10.0)?;
    c.thicken(&[c.face_at([0.0, 5.0, 5.0])?], 1.0)
}
