// yack: brep-bench case swept/ring-slid-flat-short-of-its-hole
//! A ring lying flat, slid across less than its hole.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    torus(&Frame::WORLD, 6.0, 2.0)?.swept(&Motion::slide([6.0, 0.0, 0.0]))
}
