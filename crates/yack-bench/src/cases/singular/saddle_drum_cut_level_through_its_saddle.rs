// yack: brep-bench case singular/saddle-drum-cut-level-through-its-saddle
//! A drum under a saddle, cut level through the saddle point.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    // geometry/saddle-drum: the saddle's corner heights, raised 5 so they are positive, over a slab at z = -5
    let k = [0.0, 0.0, 1.0, 1.0];
    let saddle = slab(&Frame::at([-10.0, -10.0, -5.0]), [20.0, 20.0], &k, &k, &[vec![27.5, 2.5], vec![2.5, 27.5]])?;
    let drum = saddle.intersection(&cylinder(&Frame::WORLD, 8.0, 30.0)?)?;
    Ok(drum.split(&Plane::new([0.0, 0.0, 10.0], [0.0, 0.0, 1.0])?)?.back)
}
