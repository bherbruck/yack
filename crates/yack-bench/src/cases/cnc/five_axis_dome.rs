// yack: brep-bench case cnc/five-axis-dome
//! A dome machined in five axes by four cutters in 238 passes: roughed in levels, finished along
//! its meridians and round its pole, spot-faced and drilled square to it.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    // The program as data, as the cutter's rigid motion relative to the part: slides, turns about the z axis and
    // turns about level lines through the origin; each tool is made at its first pose, its axis along it.
    let n = |lat: f64, lon: f64| Dir3::spherical(deg(lat), deg(lon)).vec();
    let slide = |v: Vec3, by: f64| Motion::slide(v * by);
    let about = |axis: Vec3, by: f64| Motion::turn(Point3::ORIGIN, axis, rad(0.0), deg(by));
    let (up, level) = (Vec3::new(0.0, 0.0, 1.0), Vec3::new(0.0, -1.0, 0.0));
    // 1. roughing: at each level the circles of radius rho + 9 + 12m below 58, largest first
    let mut circles = Vec::new();
    for z in [41.0f64, 36.0, 31.0, 26.0, 21.0, 16.0, 11.0, 6.0] {
        let rho = (1600.0 - z * z).max(0.0).sqrt();
        let radii: Vec<f64> = (0..).map(|m| rho + 9.0 + 12.0 * f64::from(m)).take_while(|r| *r < 58.0).collect();
        circles.extend(radii.into_iter().rev().map(|r| (z, r)));
    }
    let first = circles.first().map_or(0.0, |c| c.1);
    let mut rough = Vec::new();
    let mut at = first;
    for &(z, r) in &circles {
        if r != at {
            rough.push(Motion::slide(Vec3::new(r - at, 0.0, 0.0)));
        }
        rough.extend([slide(up, z - 50.0), about(up, 360.0), slide(up, 50.0 - z)]);
        at = r;
    }
    // 2. finishing: 180 meridians, 15 circles round the pole, the pole
    let mut finish = Vec::new();
    for k in 0..180 {
        let phi = (2.0 * f64::from(k)).to_radians();
        let (along, back) = (Vec3::new(phi.sin(), -phi.cos(), 0.0), Vec3::new(-phi.sin(), phi.cos(), 0.0));
        let lon = 2.0 * f64::from(k);
        finish.extend([slide(n(3.0, lon), -50.0), about(along, 72.0), slide(n(75.0, lon), 50.0)]);
        finish.extend([about(back, 72.0), about(up, 2.0)]);
    }
    finish.push(about(level, 72.0));
    for lat in (75..90).map(f64::from) {
        finish.extend([slide(n(lat, 0.0), -50.0), about(up, 360.0), slide(n(lat, 0.0), 50.0), about(level, 1.0)]);
    }
    finish.extend([slide(up, -50.0), slide(up, 50.0)]);
    // 3. and 4. spot faces and holes at the twelve points of latitude 40
    let (mut seats, mut holes) = (Vec::new(), Vec::new());
    for k in 0..12 {
        let v = n(40.0, 15.0 + 30.0 * f64::from(k));
        seats.extend([slide(v, -31.5), slide(v, 31.5), about(up, 30.0)]);
        holes.extend([slide(v, -40.0), slide(v, 40.0), about(up, 30.0)]);
    }
    let (a, s) = (n(3.0, 0.0), n(40.0, 15.0));
    let bull = Loop::starting_at([0.0, 0.0]).line_to([6.0, 0.0]).arc_ccw_to([8.0, 2.0], [6.0, 2.0]);
    let bull = Profile::new(bull.line_to([8.0, 40.0]).line_to([0.0, 40.0]).close()?, vec![])?;
    let bull = revolve(&Frame::at([first, 0.0, 50.0]), &bull, deg(360.0))?;
    let ball = Loop::starting_at([0.0, 0.0]).arc_ccw_to([4.0, 4.0], [0.0, 4.0]).line_to([4.0, 40.0]);
    let ball = Profile::new(ball.line_to([0.0, 40.0]).close()?, vec![])?;
    let ball = revolve(&Frame::along(Point3::ORIGIN + a * 90.0, a)?, &ball, deg(360.0))?;
    let pose = Frame::along(Point3::ORIGIN + s * 70.0, s)?;
    let drill = Profile::polygon(&[[0.0, 0.0], [2.5, 2.5], [2.5, 40.0], [0.0, 40.0]])?;
    let (face_mill, drill) = (cylinder(&pose, 5.0, 40.0)?, revolve(&pose, &drill, deg(360.0))?);
    let blank = cylinder(&Frame::WORLD, 50.0, 45.0)?;
    let blank = blank.cut_swept(&bull, &Motion::then(rough))?.cut_swept(&ball, &Motion::then(finish))?;
    blank.cut_swept(&face_mill, &Motion::then(seats))?.cut_swept(&drill, &Motion::then(holes))
}
