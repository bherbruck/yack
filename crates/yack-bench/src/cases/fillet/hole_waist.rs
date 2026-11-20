//! A hole whose two rim rounds meet at its waist.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let plate = block(&Frame::at([-10.0, -10.0, 0.0]), [20.0, 20.0, 2.0])?;
    let s = plate.difference(&cylinder(&Frame::at([0.0, 0.0, -1.0]), 3.0, 4.0)?)?;
    s.fillet(&[s.edge_at([0.0, 3.0, 2.0])?, s.edge_at([0.0, 3.0, 0.0])?], 1.0)
}
