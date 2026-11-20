// yack: brep-bench case nurbs/lens-of-two-nurbs-balls
//! The lens common to two NURBS balls.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let c = [[1., 0.], [1., 1.], [0., 1.], [-1., 1.], [-1., 0.], [-1., -1.], [0., -1.], [1., -1.], [1., 0.]];
    let m = [[0., -5.], [5., -5.], [5., 0.], [5., 5.], [0., 5.]];
    let p: Vec<Vec<_>> = c.iter().map(|&[x, y]| m.iter().map(|&[r, z]| [r * x, r * y, z].into()).collect()).collect();
    let w: Vec<Vec<f64>> = (0..9).map(|i| (0..5).map(|j| 0.5f64.sqrt().powi(i % 2 + j % 2)).collect()).collect();
    let (ku, kv) = ([0., 0., 0., 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1., 1., 1.], [0., 0., 0., 0.5, 0.5, 1., 1., 1.]);
    let a = nurbs_solid(&Frame::at([-3.0, 0.0, 0.0]), [2, 2], [&ku, &kv], &p, Some(&w))?;
    a.intersection(&nurbs_solid(&Frame::at([3.0, 0.0, 0.0]), [2, 2], [&ku, &kv], &p, Some(&w))?)
}
