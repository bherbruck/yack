// yack: brep-bench case geometry/nurbs-sphere
//! A ball bounded by one NURBS surface with two poles.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let c = [[1.0, 0.0], [1.0, 1.0], [0.0, 1.0], [-1.0, 1.0], [-1.0, 0.0], [-1.0, -1.0], [0.0, -1.0], [1.0, -1.0]];
    let (m, w) = ([[0.0, -5.0], [5.0, -5.0], [5.0, 0.0], [5.0, 5.0], [0.0, 5.0]], [1.0, 0.5f64.sqrt()]);
    let net = c.iter().chain(c.first()).map(|&[x, y]| m.map(|[r, z]| Point3::from([r * x, r * y, z])).to_vec());
    let weights: Vec<Vec<f64>> = (0..9).map(|i| (0..5).map(|j| w[i % 2] * w[j % 2]).collect()).collect();
    let ku = [0.0, 0.0, 0.0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1.0, 1.0, 1.0];
    let kv = [0.0, 0.0, 0.0, 0.5, 0.5, 1.0, 1.0, 1.0];
    nurbs_solid(&Frame::WORLD, [2, 2], [&ku, &kv], &net.collect::<Vec<_>>(), Some(&weights))
}
