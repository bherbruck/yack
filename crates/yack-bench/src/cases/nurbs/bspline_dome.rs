// yack: brep-bench case nurbs/bspline-dome
//! A block under a bicubic B-spline with a double knot.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let (x, y) = ([0.0, 3.0, 7.0, 11.0, 16.0, 20.0], [0.0, 5.0, 10.0, 15.0, 20.0]);
    let z = [[3., 5., 6., 5., 4.], [5., 9., 11., 8., 5.], [6., 12., 15., 10., 6.], [4., 10., 18., 9., 5.]];
    let rows = x.iter().zip(z.iter().chain(&[[5., 8., 9., 7., 4.], [2., 4., 5., 4., 3.]]));
    let ctrl = rows.map(|(&xi, h)| y.iter().zip(h).map(|(&yj, &zij)| Point3::new(xi, yj, zij)).collect()).collect();
    let knots = [vec![0., 0., 0., 0., 0.4, 0.4, 1., 1., 1., 1.], vec![0., 0., 0., 0., 1. / 3., 1., 1., 1., 1.]];
    let b = block(&Frame::WORLD, [20.0, 20.0, 10.0])?;
    b.replace_face(b.face_at([10.0, 10.0, 10.0])?, &Patch::Bspline { degree: [3, 3], knots, ctrl, weights: None })
}
