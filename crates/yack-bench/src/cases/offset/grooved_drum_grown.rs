// yack: brep-bench case offset/grooved-drum-grown
//! A grooved drum grown until its groove closes.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let groove = revolve(&Frame::WORLD, &Profile::rectangle([7.0, 4.2], [11.0, 5.8])?, deg(360.0))?;
    cylinder(&Frame::WORLD, 10.0, 10.0)?.difference(&groove)?.offset(1.0)
}
