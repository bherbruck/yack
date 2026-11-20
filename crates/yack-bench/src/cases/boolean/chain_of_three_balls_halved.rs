// yack: brep-bench case boolean/chain-of-three-balls-halved
//! A chain of three balls halved through their centers.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let two = sphere(&Frame::WORLD, 5.0)?.union(&sphere(&Frame::at([7.0, 0.0, 0.0]), 4.0)?)?;
    let chain = two.union(&sphere(&Frame::at([10.0, 4.0, 0.0]), 3.0)?)?;
    Ok(chain.split(&Plane::new([0.0, 0.0, 0.0], [0.0, 0.0, 1.0])?)?.front)
}
