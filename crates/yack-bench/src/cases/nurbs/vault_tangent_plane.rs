// yack: brep-bench case nurbs/vault-tangent-plane
//! A weighted barrel vault cut by the plane along its crown.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let arch = Loop::starting_at([-12.0, 0.0]).line_to([12.0, 0.0]).line_to([12.0, 10.0]);
    let section = Profile::new(arch.conic_to([-12.0, 10.0], [0.0, 26.0], 0.6).close()?, vec![])?;
    let along_x = Frame::new([-12.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0])?;
    Ok(extrude(&along_x, &section, 24.0)?.split(&Plane::new([0.0, 0.0, 16.0], [0.0, 0.0, 1.0])?)?.back)
}
