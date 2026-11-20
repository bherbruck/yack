// yack: brep-bench case boolean/stepped-shaft
//! A stepped shaft.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    cylinder(&Frame::WORLD, 5.0, 10.0)?.union(&cylinder(&Frame::at([0.0, 0.0, 10.0]), 3.0, 10.0)?)
}
