// yack: brep-bench case shell/l-block-open-top
//! An L-shaped plate shelled with its top open, the wall at its inside corner rounded.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let l = Profile::polygon(&[[0.0, 0.0], [20.0, 0.0], [20.0, 5.0], [5.0, 5.0], [5.0, 15.0], [0.0, 15.0]])?;
    let plate = extrude(&Frame::WORLD, &l, 8.0)?;
    plate.shell_rounded(&[plate.face_at([2.0, 2.0, 8.0])?], 1.0)
}
