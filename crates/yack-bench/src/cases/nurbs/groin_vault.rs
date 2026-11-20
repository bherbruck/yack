// yack: brep-bench case nurbs/groin-vault
//! A groin vault: two weighted barrel vaults in common.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let arch = Loop::starting_at([-12.0, 0.0]).line_to([12.0, 0.0]).line_to([12.0, 10.0]);
    let section = Profile::new(arch.conic_to([-12.0, 10.0], [0.0, 26.0], 0.6).close()?, vec![])?;
    let along_x = Frame::new([-12.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0])?;
    let along_y = Frame::new([0.0, -12.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0])?;
    extrude(&along_x, &section, 24.0)?.intersection(&extrude(&along_y, &section, 24.0)?)
}
