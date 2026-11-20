// yack: brep-bench case parts/star-handle
//! A drafted star handle.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    // the outline as (x, z): tips r = 4 on the axes, fillets r = 3 (negative: curving in) on the diagonals
    let (t, d, r, f) = (33.1, 15.9 / 2f64.sqrt(), 4.0, -3.0);
    let c = [(t, 0., r), (d, d, f), (0., t, r), (-d, d, f), (-t, 0., r), (-d, -d, f), (0., -t, r), (d, -d, f)];
    let star = Profile::tangent_chain(&c.map(|(x, z, r)| (Point2::new(x, z), r)))?;
    // drawn in x and z, the frame's own z is −y, so the 12 along +y is a height of −12
    let plane = Frame::new([0.0; 3], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0])?;
    extrude_drafted(&plane, &star, -12.0, deg(2.0), false)
}
