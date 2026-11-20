// yack: brep-bench case draft/pocket-walls
//! A pocket with its walls drafted.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let pocket = block(&Frame::at([8.0, 8.0, 5.0]), [24.0, 14.0, 11.0])?;
    let b = block(&Frame::WORLD, [40.0, 30.0, 15.0])?.difference(&pocket)?;
    let walls = b.faces_where(|f| (4.0..6.0).contains(&f.bbox().min.z) && f.bbox().max.z > 14.0)?;
    b.draft(&walls, [0.0, 0.0, 5.0], [0.0, 0.0, 1.0], deg(5.0))
}
