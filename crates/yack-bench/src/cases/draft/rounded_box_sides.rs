// yack: brep-bench case draft/rounded-box-sides
//! A rounded box with its sides drafted, the rounds with them.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let b = block(&Frame::WORLD, [30.0, 20.0, 10.0])?;
    let corners = [[0.0, 0.0, 5.0], [30.0, 0.0, 5.0], [30.0, 20.0, 5.0], [0.0, 20.0, 5.0]];
    let b = b.fillet(&corners.map(|p| b.edge_at(p)).into_iter().collect::<Result<Vec<_>>>()?, 4.0)?;
    let sides = b.faces_where(|f| f.bbox().max.z - f.bbox().min.z > 1.0);
    b.draft(&sides, [0.0, 0.0, 0.0], [0.0, 0.0, 1.0], deg(5.0))
}
