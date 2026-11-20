// yack: brep-bench case edit/blind-bore-floor-bevel-deleted
//! A blind bore's beveled floor deleted: the floor and the wall meet again.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let cutter = cylinder(&Frame::at([0.0, 0.0, 5.0]), 4.0, 6.0)?;
    let cutter = cutter.union(&cone(&Frame::at([0.0, 0.0, 4.0]), 3.0, 4.0, 1.0)?)?;
    let part = block(&Frame::at([-10.0, -10.0, 0.0]), [20.0, 20.0, 10.0])?.difference(&cutter)?;
    part.delete_faces(&[part.face_at([0.0, 3.5, 4.5])?])
}
