// yack: brep-bench case offset/slant-cut-cylinder-grown-by-a-half
//! A cylinder cut by a plane at 45° and grown by a half.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let post = cylinder(&Frame::at([0.0, 0.0, -10.0]), 3.0, 20.0)?;
    post.split(&Plane::new([0.0, 0.0, 0.0], [-1.0, 0.0, 1.0])?)?.back.offset(0.5)
}
