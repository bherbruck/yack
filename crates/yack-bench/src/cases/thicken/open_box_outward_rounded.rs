// yack: brep-bench case thicken/open-box-outward-rounded
//! An open box's sheet thickened outward, its edges rounded.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let c = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    let tray = c.faces_at(&[[5.0, 5.0, 0.0], [0.0, 5.0, 5.0], [10.0, 5.0, 5.0], [5.0, 0.0, 5.0], [5.0, 10.0, 5.0]])?;
    c.thicken(&tray, 1.0, Join::Round)
}
