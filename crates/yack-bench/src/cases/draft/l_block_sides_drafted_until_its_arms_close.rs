// yack: brep-bench case draft/l-block-sides-drafted-until-its-arms-close
//! An L-block with its sides drafted until they close over its arms.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let outline = Profile::polygon(&[[0.0, 0.0], [20.0, 0.0], [20.0, 5.0], [5.0, 5.0], [5.0, 15.0], [0.0, 15.0]])?;
    let l = extrude(&Frame::WORLD, &outline, 8.0)?;
    let sides = l.faces_where(|f| f.bbox().max.z - f.bbox().min.z > 1.0);
    l.draft(&sides, [0.0, 0.0, 0.0], [0.0, 0.0, 1.0], deg(30.0))
}
