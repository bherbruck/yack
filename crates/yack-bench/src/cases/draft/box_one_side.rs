// yack: brep-bench case draft/box-one-side
//! A box with one side drafted.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let b = block(&Frame::WORLD, [20.0, 12.0, 10.0])?;
    b.draft(&[b.face_at([20.0, 6.0, 5.0])?], [0.0, 0.0, 0.0], [0.0, 0.0, 1.0], deg(10.0))
}
