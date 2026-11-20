// yack: brep-bench case draft/t-block-sides-drafted-until-its-bar-and-stem-close
//! A T-block with its sides drafted until they close over its bar and its stem.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let t = [[0.0, 0.0], [20.0, 0.0], [20.0, 4.0], [13.0, 4.0], [13.0, 16.0], [7.0, 16.0], [7.0, 4.0], [0.0, 4.0]];
    let part = extrude(&Frame::WORLD, &Profile::polygon(&t)?, 8.0)?;
    let sides = part.faces_where(|f| f.bbox().max.z - f.bbox().min.z > 1.0)?;
    part.draft(&sides, [0.0, 0.0, 0.0], [0.0, 0.0, 1.0], deg(45.0))
}
