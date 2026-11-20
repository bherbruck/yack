// yack: brep-bench case shell/rounded-puck-hollowed-past-its-rounds
//! A rounded puck hollowed with walls thicker than its rounds.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let p = cylinder(&Frame::WORLD, 6.0, 10.0)?;
    p.fillet(&[p.edge_at([0.0, -6.0, 0.0])?, p.edge_at([0.0, -6.0, 10.0])?], 2.0)?.shell(&[], 3.0)
}
