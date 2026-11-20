// yack: brep-bench case boolean/half-torus
//! Half a torus.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let ring = torus(&Frame::WORLD, 10.0, 2.0)?;
    Ok(ring.split(&Plane::new([0.0, 0.0, 0.0], [1.0, 0.0, 0.0])?)?.front)
}
