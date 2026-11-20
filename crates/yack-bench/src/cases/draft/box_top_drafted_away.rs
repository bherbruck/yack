// yack: brep-bench case draft/box-top-drafted-away
//! A box drafted until its top is gone.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let b = block(&Frame::WORLD, [30.0, 10.0, 10.0])?;
    let sides = b.faces_where(|f| f.bbox().max.z - f.bbox().min.z > 1.0);
    b.draft(&sides, [0.0, 0.0, 0.0], [0.0, 0.0, 1.0], deg(30.0))
}
