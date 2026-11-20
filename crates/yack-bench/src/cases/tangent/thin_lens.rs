// yack: brep-bench case tangent/thin-lens
//! A lens a millionth thick.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    sphere(&Frame::WORLD, 5.0)?.intersection(&sphere(&Frame::at([9.999999, 0.0, 0.0]), 5.0)?)
}
