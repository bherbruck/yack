// yack: brep-bench case nurbs/flat-bspline-slabs-united
//! Two slabs united along flat B-spline faces in one plane.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    // each slab's patch face is a Bézier net lying flat in z = 4, parametrised as the case gives it
    let net = |u: &[f64], v: &[f64]| u.iter().map(|&x| v.iter().map(|&y| Point3::new(x, y, 4.)).collect()).collect();
    let (top, bottom) = (net(&[0., 2., 7., 10.], &[0., 4., 5., 10.]), net(&[5., 12., 15.], &[3., 4., 13.]));
    let low = block(&Frame::WORLD, [10.0, 10.0, 4.0])?;
    let low = low.replace_face(low.face_at([5.0, 5.0, 4.0])?, &Patch::Bezier { ctrl: top })?;
    let high = block(&Frame::at([5.0, 3.0, 4.0]), [10.0, 10.0, 3.0])?;
    low.union(&high.replace_face(high.face_at([10.0, 8.0, 4.0])?, &Patch::Bezier { ctrl: bottom })?)
}
