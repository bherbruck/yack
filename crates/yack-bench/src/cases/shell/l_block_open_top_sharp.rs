//! An L-shaped plate shelled with its top open, the wall at its inside corner kept sharp.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let l = Profile::polygon(&[[0.0, 0.0], [20.0, 0.0], [20.0, 5.0], [5.0, 5.0], [5.0, 15.0], [0.0, 15.0]])?;
    let plate = extrude(&Frame::WORLD, &l, 8.0)?;
    plate.shell(&[plate.face_at([2.0, 2.0, 8.0])?], 1.0)
}
