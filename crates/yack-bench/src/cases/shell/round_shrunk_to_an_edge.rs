//! A plate with rounded corners shelled as thick as its rounds: the inside rounds shrink to edges.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let b = block(&Frame::WORLD, [20.0, 14.0, 10.0])?;
    let edges = [[0.0, 0.0, 5.0], [20.0, 0.0, 5.0], [20.0, 14.0, 5.0], [0.0, 14.0, 5.0]];
    let b = b.fillet(&edges.iter().map(|&p| b.edge_at(p)).collect::<Result<Vec<_>>>()?, 2.0)?;
    b.shell(&[b.face_at([10.0, 7.0, 10.0])?], 2.0)
}
