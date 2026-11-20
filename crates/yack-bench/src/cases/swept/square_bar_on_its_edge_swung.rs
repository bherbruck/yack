// yack: brep-bench case swept/square-bar-on-its-edge-swung
//! A square bar standing on its edge, swung a quarter turn.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let bar = extrude(&Frame::WORLD, &Profile::polygon(&[[4.0, 0.0], [6.0, -2.0], [8.0, 0.0], [6.0, 2.0]])?, 3.0)?;
    bar.swept(&Motion::turn([0.0; 3], [0.0, 0.0, 1.0], rad(0.0), deg(90.0)))
}
