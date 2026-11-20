// yack: brep-bench case edit/blind-bore-floor-round-deleted
//! A blind bore's rounded floor deleted: the floor and the wall meet again.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let bored = block(&Frame::at([-10.0, -10.0, 0.0]), [20.0, 20.0, 10.0])?;
    let bored = bored.difference(&cylinder(&Frame::at([0.0, 0.0, 4.0]), 4.0, 7.0)?)?;
    let rounded = bored.fillet(&[bored.edge_at([0.0, 4.0, 4.0])?], 1.0)?;
    rounded.delete_faces(&rounded.faces_where(|f| matches!(f.kind(), yack::SurfaceKind::Torus))?)
}
