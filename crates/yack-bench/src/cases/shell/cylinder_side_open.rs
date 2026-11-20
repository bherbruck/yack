// yack: brep-bench case shell/cylinder-side-open
//! A cylinder shelled open at its side: two separate disks.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let c = cylinder(&Frame::WORLD, 5.0, 10.0)?;
    c.shell(&[c.face_at([0.0, -5.0, 5.0])?], 1.0)
}
