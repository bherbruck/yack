// yack: brep-bench case draft/hole-tapered
//! A hole drafted into a taper.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let b = block(&Frame::at([-15.0, -15.0, 0.0]), [30.0, 30.0, 20.0])?;
    let b = b.difference(&cylinder(&Frame::at([0.0, 0.0, -1.0]), 6.0, 22.0)?)?;
    b.draft(&[b.face_at([0.0, 6.0, 10.0])?], [0.0, 0.0, 0.0], [0.0, 0.0, 1.0], deg(3.0))
}
