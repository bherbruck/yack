//! A cylinder cut by a plane at 30° and grown by 1.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let post = cylinder(&Frame::at([0.0, 0.0, -10.0]), 3.0, 20.0)?;
    let cut = post.split(&Plane::new([0.0, 0.0, 0.0], [-30f64.to_radians().tan(), 0.0, 1.0])?)?.back;
    cut.offset(1.0)
}
