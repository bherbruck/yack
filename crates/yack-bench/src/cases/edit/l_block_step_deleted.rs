// yack: brep-bench case edit/l-block-step-deleted
//! An L-block's step deleted: the block is whole again.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let l = Profile::polygon(&[[0.0, 0.0], [20.0, 0.0], [20.0, 5.0], [5.0, 5.0], [5.0, 15.0], [0.0, 15.0]])?;
    let b = extrude(&Frame::WORLD, &l, 8.0)?;
    b.delete_faces(&[b.face_at([5.0, 10.0, 4.0])?, b.face_at([12.0, 5.0, 4.0])?])
}
