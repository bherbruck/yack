// yack: brep-bench case offset/filleted-ball-on-plate-grown-past-its-fillet
//! A ball standing in a plate, its root filleted, grown past the fillet's radius.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let plate = cylinder(&Frame::at([0.0, 0.0, -2.0]), 8.0, 2.0)?;
    let s = plate.union(&sphere(&Frame::at([0.0, 0.0, 1.0]), 3.0)?)?;
    s.fillet(&[s.edge_at([2.0, 2.0, 0.0])?], 1.0)?.offset(2.0)
}
