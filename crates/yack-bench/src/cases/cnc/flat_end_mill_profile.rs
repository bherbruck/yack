// yack: brep-bench case cnc/flat-end-mill-profile
//! A groove profiled round a cross by a flat end mill in three step-downs.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    // the cross listed from the middle of its bottom edge (the plunge point), every corner rounded at 6; per
    // depth a plunge and an anticlockwise lap from there, then the rise to (60, 10, 5)
    let cross = [[60., 10.], [75., 10.], [75., 30.], [100., 30.], [100., 60.], [75., 60.], [75., 80.]];
    let cross = [&cross[..], &[[45., 80.], [45., 60.], [20., 60.], [20., 30.], [45., 30.], [45., 10.]]].concat();
    let path = Loop::polygon(&cross)?.rounded(6.0)?;
    let mut legs = Vec::new();
    for (z, down) in [(-3.0, -8.0), (-6.0, -3.0), (-9.0, -3.0)] {
        legs.extend([Motion::slide([0.0, 0.0, down]), Motion::along_loop(&Frame::at([0.0, 0.0, z]), &path)?]);
    }
    legs.push(Motion::slide([0.0, 0.0, 14.0]));
    let mill = cylinder(&Frame::at([60.0, 10.0, 5.0]), 3.0, 20.0)?;
    block(&Frame::at([0.0, 0.0, -15.0]), [120.0, 90.0, 15.0])?.cut_swept(&mill, &Motion::then(legs))
}
