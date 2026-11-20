// yack: brep-bench case offset/filleted-post-grown-past-its-fillet
//! A post on a plate, filleted at its foot, grown past the fillet's radius.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let s = cylinder(&Frame::at([0.0, 0.0, -4.0]), 8.0, 4.0)?.union(&cylinder(&Frame::WORLD, 3.0, 8.0)?)?;
    let h = 1.5 * 2f64.sqrt();
    s.fillet(&[s.edge_at([h, h, 0.0])?], 2.0)?.offset(3.0)
}
