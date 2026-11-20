// yack: brep-bench case draft/box-cross-drilled
//! A box drilled across, then drafted where the hole comes out.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let axis = Frame::new([-1.0, 10.0, 10.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0])?;
    let b = block(&Frame::WORLD, [30.0, 20.0, 20.0])?.difference(&cylinder(&axis, 4.0, 32.0)?)?;
    b.draft(&[b.face_at([0.0, 2.0, 2.0])?, b.face_at([30.0, 2.0, 2.0])?], [0.0, 0.0, 0.0], [0.0, 0.0, 1.0], deg(8.0))
}
