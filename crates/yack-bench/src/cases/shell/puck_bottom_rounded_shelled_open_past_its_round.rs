// yack: brep-bench case shell/puck-bottom-rounded-shelled-open-past-its-round
//! A puck with its bottom rim rounded, shelled open at its top past the round's radius.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let p = cylinder(&Frame::WORLD, 6.0, 10.0)?;
    let p = p.fillet(&[p.edge_at([0.0, -6.0, 0.0])?], 2.0)?;
    p.shell_rounded(&[p.face_at([1.0, 1.0, 10.0])?], 3.0)
}
