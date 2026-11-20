// yack: brep-bench case fillet/round-swallowing-a-tiny-face
//! A broken edge rounded over.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let broken = Profile::polygon(&[[0.0, 0.0], [10.0, 0.0], [10.0, 9.9999], [9.9999, 10.0], [0.0, 10.0]])?;
    let s = extrude(&Frame::WORLD, &broken, 5.0)?;
    s.fillet(&[s.edge_at([10.0, 9.9999, 2.5])?, s.edge_at([9.9999, 10.0, 2.5])?], 1.0)
}
