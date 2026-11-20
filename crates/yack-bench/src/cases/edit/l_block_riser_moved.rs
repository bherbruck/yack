//! An L-block's step moved out.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let l = Profile::polygon(&[[0.0, 0.0], [40.0, 0.0], [40.0, 10.0], [15.0, 10.0], [15.0, 30.0], [0.0, 30.0]])?;
    let b = extrude(&Frame::new([0.0, 20.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0])?, &l, 20.0)?;
    b.move_faces(&[b.face_at([15.0, 10.0, 20.0])?], &Rigid::translation([10.0, 0.0, 0.0]))
}
