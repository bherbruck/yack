// yack: brep-bench case offset/filleted-blind-bore-grown-past-its-fillet
//! A block with a blind bore, filleted at the bore's floor, grown past the fillet's radius.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let s = cylinder(&Frame::WORLD, 10.0, 8.0)?.difference(&cylinder(&Frame::at([0.0, 0.0, 2.0]), 5.0, 7.0)?)?;
    let h = 2.5 * 2f64.sqrt();
    s.fillet(&[s.edge_at([h, h, 2.0])?], 2.0)?.offset(3.0)
}
