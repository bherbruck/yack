//! A block's beveled edge deleted: the top and the side meet again.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let b = block(&Frame::WORLD, [20.0, 10.0, 10.0])?;
    let beveled = b.chamfer(&[b.edge_at([20.0, 5.0, 10.0])?], 3.0)?;
    beveled.delete_faces(&[beveled.face_at([18.5, 5.0, 8.5])?])
}
