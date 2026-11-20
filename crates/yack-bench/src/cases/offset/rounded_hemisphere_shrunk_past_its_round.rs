// yack: brep-bench case offset/rounded-hemisphere-shrunk-past-its-round
//! A hemisphere with its rim rounded, shrunk past the round's radius.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let s = sphere(&Frame::WORLD, 5.0)?.split(&Plane::new([0.0, 0.0, 0.0], [0.0, 0.0, 1.0])?)?.front;
    let h = 2.5 * 2f64.sqrt();
    s.fillet(&[s.edge_at([h, h, 0.0])?], 1.0)?.offset(-1.5)
}
