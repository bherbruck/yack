// yack: brep-bench case singular/cylinder-hoof
//! A hoof cut from a cylinder through a diameter of its base.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let up = Plane::new([0.0; 3], [0.0, 0.0, 1.0])?;
    let above = cylinder(&Frame::at([0.0, 0.0, -1.0]), 5.0, 11.0)?.split(&up)?.front;
    Ok(above.split(&Plane::new([0.0; 3], [-(30f64.to_radians().tan()), 0.0, 1.0])?)?.back)
}
