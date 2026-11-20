// yack: brep-bench case fillet/boss-s-curve
//! A boss whose root and rim rounds meet.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let plate = block(&Frame::at([-20.0, -20.0, 0.0]), [40.0, 40.0, 4.0])?;
    let s = plate.union(&cylinder(&Frame::at([0.0, 0.0, 4.0]), 5.0, 4.0)?)?;
    s.fillet(&[s.edge_at([0.0, 5.0, 8.0])?, s.edge_at([0.0, 5.0, 4.0])?], 2.0)
}
