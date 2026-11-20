// yack: brep-bench case swept/bar-plunged-aslant-into-plate
//! A bar plunged aslant into a plate, stopping inside.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let along = Frame::new([6.0, 0.0, -6.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0])?;
    let bar = extrude(&along, &Profile::polygon(&[[1.0, 0.0], [0.0, 1.0], [-1.0, 0.0], [0.0, -1.0]])?, 8.0)?;
    let push = Motion::slide([8.0 * 0.3, 0.0, 8.0 * 0.91f64.sqrt()]);
    block(&Frame::at([0.0, -5.0, 0.0]), [20.0, 10.0, 4.0])?.cut_swept(&bar, &push)
}
