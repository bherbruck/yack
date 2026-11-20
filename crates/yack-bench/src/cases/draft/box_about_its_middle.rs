// yack: brep-bench case draft/box-about-its-middle
//! A box drafted about its middle.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let b = block(&Frame::WORLD, [20.0, 12.0, 10.0])?;
    let sides = b.faces_where(|f| f.bbox().max.z - f.bbox().min.z > 1.0)?;
    b.draft(&sides, [0.0, 0.0, 5.0], [0.0, 0.0, 1.0], deg(5.0))
}
