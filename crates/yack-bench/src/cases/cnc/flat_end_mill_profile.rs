// yack: brep-bench case cnc/flat-end-mill-profile
//! A groove profiled round a cross by a flat end mill in three step-downs.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    // The program as data: tip positions and arc centres; every move of the upright mill is a slide (G1) or, for an
    // arc (G2/G3), a turn about the arc's upright centre line, which carries an upright round tool along the arc.
    let cross = [[45.0, 10.0], [75.0, 10.0], [75.0, 30.0], [100.0, 30.0], [100.0, 60.0], [75.0, 60.0]];
    let cross = [cross, [[75.0, 80.0], [45.0, 80.0], [45.0, 60.0], [20.0, 60.0], [20.0, 30.0], [45.0, 30.0]]].concat();
    let ring: Vec<[f64; 2]> = cross.iter().cycle().take(14).copied().collect();
    let slide = |a: [f64; 3], b: [f64; 3]| Motion::slide(Vec3::new(b[0] - a[0], b[1] - a[1], b[2] - a[2]));
    let (mut legs, mut at) = (Vec::new(), [60.0, 10.0, 5.0]);
    for z in [-3.0, -6.0, -9.0] {
        legs.push(slide(at, [60.0, 10.0, z]));
        at = [60.0, 10.0, z];
        for w in ring.windows(3) {
            let [p, c, n] = w else { continue };
            let unit = |a: &[f64; 2], b: &[f64; 2]| {
                let l = ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2)).sqrt();
                [(b[0] - a[0]) / l, (b[1] - a[1]) / l]
            };
            let (din, dout) = (unit(p, c), unit(c, n));
            let s = (din[0] * dout[1] - din[1] * dout[0]).signum();
            let arrive = [c[0] - 6.0 * din[0], c[1] - 6.0 * din[1], z];
            legs.push(slide(at, arrive));
            let centre = Point3::new(arrive[0] - 6.0 * s * din[1], arrive[1] + 6.0 * s * din[0], z);
            legs.push(Motion::turn(centre, Vec3::new(0.0, 0.0, s), rad(0.0), deg(90.0)));
            at = [c[0] + 6.0 * dout[0], c[1] + 6.0 * dout[1], z];
        }
        legs.push(slide(at, [60.0, 10.0, z]));
        at = [60.0, 10.0, z];
    }
    legs.push(slide(at, [60.0, 10.0, 5.0]));
    let mill = cylinder(&Frame::at([60.0, 10.0, 5.0]), 3.0, 20.0)?;
    block(&Frame::at([0.0, 0.0, -15.0]), [120.0, 90.0, 15.0])?.cut_swept(&mill, &Motion::then(legs))
}
