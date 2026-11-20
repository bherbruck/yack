//! A plate's hole widened.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let plate = block(&Frame::WORLD, [40.0, 30.0, 8.0])?;
    let plate = plate.difference(&cylinder(&Frame::at([20.0, 15.0, -1.0]), 5.0, 10.0)?)?;
    plate.offset_faces(&[plate.face_at([20.0, 20.0, 4.0])?], -2.0)
}
