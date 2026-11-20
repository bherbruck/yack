// yack: brep-bench case thicken/hemisphere-both-sides
//! A hemispherical sheet thickened to both sides.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let dome = sphere(&Frame::WORLD, 5.0)?.split(&Plane::new([0.0, 0.0, 0.0], [0.0, 0.0, 1.0])?)?.front;
    let sheet = dome.face_at([0.0, 3.0, 4.0])?;
    dome.thicken(&[sheet], 1.0)?.union(&dome.thicken(&[sheet], -1.0)?)
}
