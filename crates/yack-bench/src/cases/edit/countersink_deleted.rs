//! A hole's countersink deleted: the hole runs straight up to the top.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let b = block(&Frame::WORLD, [20.0, 10.0, 10.0])?.difference(&cylinder(&Frame::at([10.0, 5.0, -1.0]), 2.0, 12.0)?)?;
    let sunk = b.chamfer(&[b.edge_at([10.0, 7.0, 10.0])?], 2.0)?;
    sunk.delete_faces(&[sunk.face_at([10.0, 8.0, 9.0])?])
}
