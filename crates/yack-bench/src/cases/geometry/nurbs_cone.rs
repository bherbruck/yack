// yack: brep-bench case geometry/nurbs-cone
//! A cone bounded by a NURBS surface with a pole.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let c = [[1.0, 0.0], [1.0, 1.0], [0.0, 1.0], [-1.0, 1.0], [-1.0, 0.0], [-1.0, -1.0], [0.0, -1.0], [1.0, -1.0]];
    let ring = c.iter().chain(c.first()).map(|&[x, y]| vec![[5.0 * x, 5.0 * y, 0.0].into(), [0.0, 0.0, 10.0].into()]);
    let ku = [0.0, 0.0, 0.0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1.0, 1.0, 1.0];
    let weights: Vec<Vec<f64>> = (0..9).map(|i| vec![if i % 2 == 0 { 1.0 } else { 0.5f64.sqrt() }; 2]).collect();
    nurbs_solid(&Frame::WORLD, [2, 1], [&ku, &[0.0, 0.0, 1.0, 1.0]], &ring.collect::<Vec<_>>(), Some(&weights))
}
