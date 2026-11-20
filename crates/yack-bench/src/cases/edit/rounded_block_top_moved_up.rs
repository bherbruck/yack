//! A block's top moved up, carrying its round.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let b = block(&Frame::WORLD, [40.0, 30.0, 10.0])?;
    let rounded = b.fillet(&[b.edge_at([20.0, 30.0, 10.0])?], 3.0)?;
    let mut faces = rounded.faces_where(|f| matches!(f.kind(), yack::SurfaceKind::Cylinder { .. }));
    faces.push(rounded.face_at([20.0, 10.0, 10.0])?);
    rounded.move_faces(&faces, &Rigid::translation([0.0, 0.0, 6.0]))
}
