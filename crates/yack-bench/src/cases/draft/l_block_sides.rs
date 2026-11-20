// yack: brep-bench case draft/l-block-sides
//! An L-block with its sides drafted, about a concave edge.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let outline = Profile::polygon(&[[0.0, 0.0], [30.0, 0.0], [30.0, 10.0], [10.0, 10.0], [10.0, 25.0], [0.0, 25.0]])?;
    let l = extrude(&Frame::WORLD, &outline, 12.0)?;
    let sides = l.faces_where(|f| f.bbox().max.z - f.bbox().min.z > 1.0);
    l.draft(&sides, [0.0, 0.0, 0.0], [0.0, 0.0, 1.0], deg(6.0))
}
