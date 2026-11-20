// yack: brep-bench case shell/box-open-top-outward-sharp
//! A box shelled outward with its top open, its edges kept sharp.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let b = block(&Frame::WORLD, [10.0, 8.0, 6.0])?;
    b.shell_outward(&[b.face_at([5.0, 4.0, 6.0])?], 1.0)
}
