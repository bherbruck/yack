// yack: brep-bench case shell/box-open-top-and-front
//! A box shelled open at two faces that share an edge.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let b = block(&Frame::WORLD, [10.0, 8.0, 6.0])?;
    b.shell(&[b.face_at([5.0, 4.0, 6.0])?, b.face_at([5.0, 0.0, 3.0])?], 1.0)
}
