// yack: brep-bench case edit/rounded-frustum-top-round-deleted
//! A cone frustum's rounded top rim deleted, its rounded base kept.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let frustum = cone(&Frame::WORLD, 6.0, 3.0, 6.0)?;
    let rounded = frustum.fillet(&[frustum.edge_at([0.0, 3.0, 6.0])?, frustum.edge_at([0.0, 6.0, 0.0])?], 1.5)?;
    let (rho, z) = ((7.5 - 1.5 * 5f64.sqrt()) / 2.0 + 1.5 * 0.5f64.sqrt(), 4.5 + 1.5 * 0.5f64.sqrt());
    rounded.delete_faces(&[rounded.face_at([0.0, rho, z])?])
}
