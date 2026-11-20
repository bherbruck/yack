// yack: brep-bench case fillet/l-block-inside-consumed
//! An L-block whose inside corner is rounded away.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let section = Profile::polygon(&[[0.0, 0.0], [10.0, 0.0], [10.0, 3.0], [3.0, 3.0], [3.0, 10.0], [0.0, 10.0]])?;
    let s = extrude(&Frame::new([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])?, &section, 10.0)?;
    s.fillet(&[s.edge_at([3.0, 5.0, 3.0])?], 7.0)
}
