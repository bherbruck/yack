// yack: brep-bench case offset/flanged-drum-shrunk
//! A flanged drum shrunk until its flange is gone.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let drum = cylinder(&Frame::WORLD, 8.0, 6.0)?;
    drum.union(&cylinder(&Frame::at([0.0, 0.0, 2.2]), 11.0, 1.6)?)?.offset(-1.0)
}
