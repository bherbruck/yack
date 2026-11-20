// yack: brep-bench case shell/cylinder-open-top
//! A cylinder shelled with its top open: a cup.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let c = cylinder(&Frame::WORLD, 5.0, 10.0)?;
    c.shell(&[c.face_at([1.0, 1.0, 10.0])?], 1.0)
}
