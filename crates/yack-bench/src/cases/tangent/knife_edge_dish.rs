// yack: brep-bench case tangent/knife-edge-dish
//! A cylinder with a knife-edged dish.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    cylinder(&Frame::WORLD, 5.0, 10.0)?.difference(&sphere(&Frame::at([0.0, 0.0, 10.0]), 5.0)?)
}
