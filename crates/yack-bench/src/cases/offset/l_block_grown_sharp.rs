//! An L-shaped plate grown by 1, its edges kept sharp.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let l = Profile::polygon(&[[0.0, 0.0], [20.0, 0.0], [20.0, 5.0], [5.0, 5.0], [5.0, 15.0], [0.0, 15.0]])?;
    let plate = extrude(&Frame::WORLD, &l, 8.0)?;
    plate.offset_faces(&plate.faces(), 1.0)
}
