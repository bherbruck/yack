// yack: brep-bench case offset/filleted-cone-boss-grown-by-its-fillet
//! A filleted cone boss grown by its fillet's radius.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let s = cylinder(&Frame::at([0.0, 0.0, -2.0]), 8.0, 2.0)?.union(&cone(&Frame::WORLD, 4.0, 1.0, 6.0)?)?;
    let h = 2.0 * 2f64.sqrt();
    s.fillet(&[s.edge_at([h, h, 0.0])?], 1.0)?.offset(1.0)
}
