//! A ball cut flat above its middle, its rim chamfered with two distances.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let s = sphere(&Frame::WORLD, 5.0)?.split(&Plane::new([0.0, 0.0, 3.0], [0.0, 0.0, 1.0])?)?.back;
    s.chamfer_two(s.edge_at([0.0, 4.0, 3.0])?, s.face_at([1.0, 1.0, 3.0])?, 2.0, 1.0)
}
