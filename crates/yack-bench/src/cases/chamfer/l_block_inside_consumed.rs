// yack: brep-bench case chamfer/l-block-inside-consumed
//! An L-block whose inside corner is chamfered away.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let section = Profile::polygon(&[[0.0, 0.0], [10.0, 0.0], [10.0, 3.0], [3.0, 3.0], [3.0, 10.0], [0.0, 10.0]])?;
    let s = extrude(&Frame::new([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])?, &section, 10.0)?;
    s.chamfer(&[s.edge_at([3.0, 5.0, 3.0])?], 7.0)
}
