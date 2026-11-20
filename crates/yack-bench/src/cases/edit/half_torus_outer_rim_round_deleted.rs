// yack: brep-bench case edit/half-torus-outer-rim-round-deleted
//! A half torus's rounded outer rim deleted: the flat and the torus meet again.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let half = torus(&Frame::WORLD, 6.0, 2.0)?.split(&Plane::new([0.0, 0.0, 0.0], [0.0, 0.0, 1.0])?)?.front;
    let rounded = half.fillet(&[half.edge_at([0.0, 8.0, 0.0])?], 0.5)?;
    // a point of the round, off its bisector (the round's centre circle is at radius 6 + √2, z = 0.5)
    rounded.delete_faces(&[rounded.face_at([0.0, 7.794515558701765, 0.1753919415842729])?])
}
