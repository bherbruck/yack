// yack: brep-bench case geometry/spindle-torus
//! A spindle torus.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    spindle_torus(&Frame::WORLD, 3.0, 5.0)
}
