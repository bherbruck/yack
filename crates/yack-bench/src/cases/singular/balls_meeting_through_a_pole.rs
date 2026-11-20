// yack: brep-bench case singular/balls-meeting-through-a-pole
//! Two balls meeting through a pole of each.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    sphere(&Frame::WORLD, 5.0)?.union(&sphere(&Frame::at([4.8, 0.0, 3.6]), 5.0)?)
}
