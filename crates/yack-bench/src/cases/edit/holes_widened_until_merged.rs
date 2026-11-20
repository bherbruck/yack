//! Two holes widened until they merge.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let left = cylinder(&Frame::at([15.0, 15.0, -1.0]), 3.0, 10.0)?;
    let right = cylinder(&Frame::at([25.0, 15.0, -1.0]), 3.0, 10.0)?;
    let plate = block(&Frame::WORLD, [40.0, 30.0, 8.0])?.difference_all(&[left, right])?;
    plate.offset_faces(&[plate.face_at([15.0, 18.0, 4.0])?, plate.face_at([25.0, 18.0, 4.0])?], -3.0)
}
