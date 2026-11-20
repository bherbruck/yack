// yack: brep-bench case geometry/torus-nearly-horn
//! A torus whose hole is nearly closed.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    torus(&Frame::WORLD, 5.0, 4.99)
}
