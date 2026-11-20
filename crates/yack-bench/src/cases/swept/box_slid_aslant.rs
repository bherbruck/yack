// yack: brep-bench case swept/box-slid-aslant
//! A box slid aslant.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    block(&Frame::WORLD, [10.0, 6.0, 4.0])?.swept(&Motion::slide([8.0, 5.0, 3.0]))
}
