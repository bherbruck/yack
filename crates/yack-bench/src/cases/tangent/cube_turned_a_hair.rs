// yack: brep-bench case tangent/cube-turned-a-hair
//! A cube united with itself turned a hair.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let turn = Rigid::rotation([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], deg(0.001))?;
    let cube = block(&Frame::at([-5.0, -5.0, 0.0]), [10.0, 10.0, 10.0])?;
    cube.union(&block(&Frame::at([-5.0, -5.0, 0.0]).moved(&turn), [10.0, 10.0, 10.0])?)
}
