// yack: brep-bench case fill/coons-patch-of-four-lines
//! A block capped by the Coons patch of four straight edges.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    // the case allows the patch as the hyperbolic paraboloid it is: a bilinear top on the corners' heights
    let k = [0.0, 0.0, 1.0, 1.0];
    slab(&Frame::WORLD, [10.0, 10.0], &k, &k, &[vec![5.0, 5.0], vec![5.0, 15.0]])
}
