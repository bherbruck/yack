// yack: brep-bench case shell/l-block-open-at-its-step-sharp
//! An L-shaped block shelled inward open at its step, the wall rising past the opening kept.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let l = Profile::polygon(&[[0.0, 0.0], [20.0, 0.0], [20.0, 20.0], [15.0, 20.0], [15.0, 5.0], [0.0, 5.0]])?;
    let b = extrude(&Frame::WORLD, &l, 20.0)?;
    b.shell(&[b.face_at([7.0, 5.0, 10.0])?], 1.0)
}
