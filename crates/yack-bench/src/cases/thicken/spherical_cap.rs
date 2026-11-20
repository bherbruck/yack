// yack: brep-bench case thicken/spherical-cap
//! A spherical cap thickened outward, its rim a cone.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let cap = sphere(&Frame::WORLD, 10.0)?.split(&Plane::new([0.0, 0.0, 6.0], [0.0, 0.0, 1.0])?)?.front;
    cap.thicken(&[cap.face_at([0.0, 6.0, 8.0])?], 2.0)
}
