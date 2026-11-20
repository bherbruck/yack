//! A cylinder's beveled rim deleted: the top and the side meet again.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let c = cylinder(&Frame::WORLD, 6.0, 10.0)?;
    let beveled = c.chamfer(&[c.edge_at([0.0, 6.0, 10.0])?], 2.0)?;
    beveled.delete_faces(&[beveled.face_at([0.0, 5.0, 9.0])?])
}
