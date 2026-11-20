//! A post standing out of a ball, rounded where it leaves.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let s = sphere(&Frame::WORLD, 10.0)?.union(&cylinder(&Frame::WORLD, 4.0, 15.0)?)?;
    s.fillet(&[s.edge_at([0.0, 4.0, 84f64.sqrt()])?], 2.0)
}
