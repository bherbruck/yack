// yack: brep-bench case fill/coons-patch-of-four-parabolas
//! A block capped by the Coons patch of four parabolas.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    // the four arcs, each from one corner of the square (at z = 10) through its middle control point to the next
    let corners = [[0., 0.], [10., 0.], [10., 10.], [0., 10.], [0., 0.]];
    let mids = [[5., 0., 14.], [10., 5., 6.], [5., 10., 6.], [0., 5., 14.]];
    let ends = corners.iter().zip(corners.iter().skip(1));
    let arcs = mids.iter().zip(ends).map(|(m, ([a, b], [c, d]))| Spline3::bezier(&[[*a, *b, 10.], *m, [*c, *d, 10.]]));
    let b = block(&Frame::WORLD, [10.0; 3])?;
    b.replace_face(b.face_at([5.0, 5.0, 10.0])?, &Patch::Fill { boundary: arcs.collect::<Result<Vec<_>>>()? })
}
