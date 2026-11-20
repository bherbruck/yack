// yack: brep-bench case thicken/hemisphere-to-its-center
//! A hemispherical sheet thickened to its center.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let dome = sphere(&Frame::WORLD, 5.0)?.split(&Plane::new([0.0, 0.0, 0.0], [0.0, 0.0, 1.0])?)?.front;
    dome.thicken(&[dome.face_at([0.0, 3.0, 4.0])?], -5.0)
}
