// yack: brep-bench case draft/cylinder-parted-both-ways
//! A cylinder drafted both ways from a parting line.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let (top, bottom) = (cylinder(&Frame::at([0.0, 0.0, 10.0]), 10.0, 10.0)?, cylinder(&Frame::WORLD, 10.0, 10.0)?);
    let up = top.draft(&[top.face_at([0.0, 10.0, 15.0])?], [0.0, 0.0, 10.0], [0.0, 0.0, 1.0], deg(3.0))?;
    up.union(&bottom.draft(&[bottom.face_at([0.0, 10.0, 5.0])?], [0.0, 0.0, 10.0], [0.0, 0.0, -1.0], deg(3.0))?)
}
