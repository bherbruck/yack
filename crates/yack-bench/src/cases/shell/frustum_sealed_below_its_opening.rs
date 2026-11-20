// yack: brep-bench case shell/frustum-sealed-below-its-opening
//! A cone frustum shelled open at its top, the cavity closing to a point below it.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let f = cone(&Frame::WORLD, 6.0, 3.0, 4.0)?;
    f.shell(&[f.face_at([1.0, 1.0, 4.0])?], 2.7)
}
