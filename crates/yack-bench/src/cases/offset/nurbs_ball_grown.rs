// yack: brep-bench case offset/nurbs-ball-grown
//! A NURBS ball grown.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let c = [[1., 0.], [1., 1.], [0., 1.], [-1., 1.], [-1., 0.], [-1., -1.], [0., -1.], [1., -1.], [1., 0.]];
    let m = [[0., -5.], [5., -5.], [5., 0.], [5., 5.], [0., 5.]];
    let row = |&[x, y]: &[f64; 2]| m.iter().map(|&[r, z]| Point3::from([r * x, r * y, z])).collect::<Vec<_>>();
    let p: Vec<_> = c.iter().map(row).collect();
    let w: Vec<Vec<f64>> = (0..9).map(|i| (0..5).map(|j| 0.5f64.sqrt().powi(i % 2 + j % 2)).collect()).collect();
    let (ku, kv) = ([0., 0., 0., 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1., 1., 1.], [0., 0., 0., 0.5, 0.5, 1., 1., 1.]);
    nurbs_solid(&Frame::WORLD, [2, 2], [&ku, &kv], &p, Some(&w))?.offset(2.0)
}
