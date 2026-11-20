// yack: brep-bench case loft/twisted-tapering-square
//! A square tapering through a half turn on four spiral guides.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    // each corner's guide: from 15√2 off the axis at 45° + 90°k in to 5√2, turning half a turn over the 40 of height
    let (r0, r1) = (15.0 * 2f64.sqrt(), 5.0 * 2f64.sqrt());
    let guides = [45.0, 135.0, 225.0, 315.0].map(|a| {
        Spline3::conical_helix([0.0; 3], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], r0, r1, 40.0, deg(a), 0.5)
    });
    let (foot, top) = (Profile::rectangle([-15.0, -15.0], [15.0, 15.0])?, Profile::rectangle([-5.0, -5.0], [5.0, 5.])?);
    let ends = [(Frame::WORLD, foot), (Frame::at([0.0, 0.0, 40.0]).turned(deg(180.0)), top)];
    loft_guided(&ends, &guides.into_iter().collect::<Result<Vec<_>>>()?)
}
