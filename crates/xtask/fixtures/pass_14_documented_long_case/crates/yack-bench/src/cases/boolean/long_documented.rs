// yack: brep-bench case boolean/long-documented
//! Long.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let a = 1.0;
    let b = 2.0;
    let c = 3.0;
    let d = 4.0;
    let e = 5.0;
    let f = 6.0;
    let g = 7.0;
    block(&Frame::WORLD, [a + b + c + d, e + f, g])
}
