// yack: brep-bench case draft/boss-on-plate
//! A boss on a plate, its side drafted.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let plate = block(&Frame::at([-20.0, -20.0, 0.0]), [40.0, 40.0, 5.0])?;
    let part = plate.union(&cylinder(&Frame::at([0.0, 0.0, 5.0]), 8.0, 12.0)?)?;
    part.draft(&[part.face_at([0.0, 8.0, 11.0])?], [0.0, 0.0, 5.0], [0.0, 0.0, 1.0], deg(4.0))
}
