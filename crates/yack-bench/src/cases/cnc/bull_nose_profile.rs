// yack: brep-bench case cnc/bull-nose-profile
//! A groove profiled round a six-lobed flower by a bull-nose mill in three step-downs.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    // The program as data: the flower's lobe and valley centres and each arc's sweep; each G3/G2 arc of the upright,
    // round mill is a turn about the arc's upright centre line (anticlockwise round a lobe, clockwise round a valley).
    let about = |r: f64, a: f64| [60.0 + r * a.to_radians().cos(), 60.0 + r * a.to_radians().sin()];
    let toward = |a: [f64; 2], b: [f64; 2]| (b[1] - a[1]).atan2(b[0] - a[0]).to_degrees();
    let lobe = |k: i32| about(30.0, 60.0 * f64::from(k));
    let valley = |k: i32| about(40.0, 30.0 + 60.0 * f64::from(k));
    let turn = |c: [f64; 2], z: f64, up: f64, by: f64| Motion::turn([c[0], c[1], z], [0.0, 0.0, up], rad(0.0), deg(by));
    let mut legs = Vec::new();
    for (z, drop) in [(-4.0, 9.0), (-8.0, 4.0), (-10.0, 2.0)] {
        legs.push(Motion::slide(Vec3::new(0.0, 0.0, -drop)));
        for k in 0..6 {
            let (l, v, next) = (lobe(k), valley(k), lobe(k + 1));
            let from = if k == 0 { 0.0 } else { toward(l, valley(k - 1)) };
            legs.push(turn(l, z, 1.0, (toward(l, v) - from).rem_euclid(360.0)));
            legs.push(turn(v, z, -1.0, (toward(v, l) - toward(v, next)).rem_euclid(360.0)));
        }
        legs.push(turn(lobe(0), z, 1.0, (0.0 - toward(lobe(0), valley(5))).rem_euclid(360.0)));
    }
    legs.push(Motion::slide(Vec3::new(0.0, 0.0, 15.0)));
    // The bull-nose mill, its tip at (102, 60, 5): flat end radius 2.5, corner radius 1.5, side radius 4, 25 long.
    let side = Loop::starting_at([0.0, 0.0]).line_to([2.5, 0.0]).arc_ccw_to([4.0, 1.5], [2.5, 1.5]);
    let side = side.line_to([4.0, 25.0]).line_to([0.0, 25.0]).close()?;
    let mill = revolve(&Frame::at([102.0, 60.0, 5.0]), &Profile::new(side, vec![])?, deg(360.0))?;
    block(&Frame::at([0.0, 0.0, -16.0]), [120.0, 120.0, 16.0])?.cut_swept(&mill, &Motion::then(legs))
}
