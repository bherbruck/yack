// yack: brep-bench case shell/rounded-hemisphere-hollowed-past-its-round
//! A hemisphere with its rim rounded, hollowed with walls thicker than the round.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let dome = sphere(&Frame::WORLD, 5.0)?.split(&Plane::new([0.0, 0.0, 0.0], [0.0, 0.0, 1.0])?)?.front;
    dome.fillet(&[dome.edge_at([0.0, -5.0, 0.0])?], 1.0)?.shell(&[], 2.0)
}
