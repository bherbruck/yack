// yack: brep-bench case geometry/horn-torus
//! A horn torus.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    spindle_torus(&Frame::WORLD, 5.0, 5.0)
}
