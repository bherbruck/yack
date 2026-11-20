// yack: brep-bench case swept/tilted-box-pushed-aslant-through-plate
//! A tilted box pushed aslant wholly through a plate.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let (e1, e2) = ([2.0 / 3.0, 1.0 / 3.0, 2.0 / 3.0], [-2.0 / 3.0, 2.0 / 3.0, 1.0 / 3.0]);
    let e3 = [-1.0 / 3.0, -2.0 / 3.0, 2.0 / 3.0];
    let center = Frame::new([-2.0, -1.0, -8.0], e1, e2, e3)?;
    let tilted = block(&Frame::new(center.point([-2.0, -1.5, -1.0]), e1, e2, e3)?, [4.0, 3.0, 2.0])?;
    let plate = block(&Frame::at([-15.0, -15.0, 0.0]), [30.0, 30.0, 4.0])?;
    plate.cut_swept(&tilted, &Motion::slide([4.0, 2.0, 18.0]))
}
