//! A ball standing in a plate, its beveled root deleted: the plate and the ball meet again.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let plate = cylinder(&Frame::at([0.0, 0.0, -2.0]), 8.0, 2.0)?.union(&sphere(&Frame::at([0.0, 0.0, 1.0]), 3.0)?)?;
    let part = plate.union(&cone(&Frame::WORLD, 4.0, 3.0, 1.0)?)?;
    part.delete_faces(&[part.face_at([0.0, 3.5, 0.5])?])
}
