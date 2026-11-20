//! A half torus's rounded outer rim deleted: the flat and the torus meet again.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let half = torus(&Frame::WORLD, 6.0, 2.0)?.split(&Plane::new([0.0, 0.0, 0.0], [0.0, 0.0, 1.0])?)?.front;
    let rounded = half.fillet(&[half.edge_at([0.0, 8.0, 0.0])?], 0.5)?;
    let (a, b) = (2f64.sqrt() + 0.125f64.sqrt(), 0.5 - 0.125f64.sqrt());
    rounded.delete_faces(&[rounded.face_at([0.0, 6.0 + a, b])?])
}
