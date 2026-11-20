//! A ball standing in a plate, its rounded root deleted: the plate and the ball meet again.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let plate = cylinder(&Frame::at([0.0, 0.0, -2.0]), 8.0, 2.0)?.union(&sphere(&Frame::at([0.0, 0.0, 1.0]), 3.0)?)?;
    let rounded = plate.fillet(&[plate.edge_at([0.0, 8f64.sqrt(), 0.0])?], 1.0)?;
    rounded.delete_faces(&rounded.faces_where(|f| matches!(f.kind(), yack::SurfaceKind::Torus)))
}
