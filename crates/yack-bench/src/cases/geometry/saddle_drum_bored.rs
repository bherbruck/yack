// yack: brep-bench case geometry/saddle-drum-bored
//! A drum under a saddle, bored through.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    // the saddle's corner heights, raised 5 so they are positive, over a slab standing at z = -5
    let k = [0.0, 0.0, 1.0, 1.0];
    let saddle = slab(&Frame::at([-10.0, -10.0, -5.0]), [20.0, 20.0], &k, &k, &[vec![27.5, 2.5], vec![2.5, 27.5]])?;
    let drum = saddle.intersection(&cylinder(&Frame::WORLD, 8.0, 30.0)?)?;
    drum.difference(&cylinder(&Frame::at([0.0, 0.0, -1.0]), 3.0, 30.0)?)
}
