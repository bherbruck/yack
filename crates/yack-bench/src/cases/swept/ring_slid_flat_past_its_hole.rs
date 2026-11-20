// yack: brep-bench case swept/ring-slid-flat-past-its-hole
//! A ring lying flat, slid across farther than its hole.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    torus(&Frame::WORLD, 6.0, 2.0)?.swept(&Motion::Slide { by: [15.0, 0.0, 0.0].into() })
}
