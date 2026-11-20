// yack: brep-bench case cnc/pocket-with-an-island
//! A 2.5-axis program of three tools: a pocket round an island cleared in raster passes, a bore and
//! four drilled holes.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    // The program as data: each tool's tip positions, `None` z for "stay"; arcs as (centre, up, degrees). Every move
    // of these upright round tools is a slide (G1) or, for an arc, a turn about its upright centre line.
    enum Step {
        To([f64; 3]),
        Arc([f64; 2], f64, f64),
    }
    let rounded = |corners: &[[f64; 2]], r: f64, z: f64, steps: &mut Vec<Step>| {
        let ring: Vec<[f64; 2]> = corners.iter().cycle().take(corners.len() + 2).copied().collect();
        for w in ring.windows(3) {
            let [p, c, n] = w else { continue };
            let unit = |a: &[f64; 2], b: &[f64; 2]| {
                let l = ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2)).sqrt();
                [(b[0] - a[0]) / l, (b[1] - a[1]) / l]
            };
            let (din, dout) = (unit(p, c), unit(c, n));
            let s = (din[0] * dout[1] - din[1] * dout[0]).signum();
            let arrive = [c[0] - r * din[0], c[1] - r * din[1]];
            steps.push(Step::To([arrive[0], arrive[1], z]));
            steps.push(Step::Arc([arrive[0] - r * s * din[1], arrive[1] + r * s * din[0]], s, 90.0));
        }
    };
    // the raster intervals of the line y across the region the mill's tip keeps to
    let intervals = |y: f64| {
        let inset = |d: f64| 32.0 - (49.0 - d * d).sqrt();
        let left = if y < 32.0 { inset(32.0 - y) } else if y > 78.0 { inset(y - 78.0) } else { 25.0 };
        let off = (y - 55.0).abs();
        let half = (400.0 - off * off).max(0.0).sqrt();
        if off < 20.0 { vec![(left, 60.0 - half), (60.0 + half, 160.0 - left)] } else { vec![(left, 160.0 - left)] }
    };
    let mut pocket = Vec::new();
    for z in [-4.0, -8.0, -12.0] {
        for y in (0..15).map(|j| 26.0 + 4.0 * f64::from(j)) {
            for (a, b) in intervals(y) {
                pocket.extend([Step::To([a, y, 5.0]), Step::To([a, y, z]), Step::To([b, y, z]), Step::To([b, y, 5.0])]);
            }
        }
        pocket.extend([Step::To([80.0, 25.0, 5.0]), Step::To([80.0, 25.0, z])]);
        rounded(&[[25.0, 25.0], [135.0, 25.0], [135.0, 85.0], [25.0, 85.0]], 7.0, z, &mut pocket);
        pocket.extend([Step::To([80.0, 25.0, z]), Step::To([80.0, 25.0, 5.0]), Step::To([80.0, 55.0, 5.0])]);
        pocket.extend([Step::To([80.0, 55.0, z]), Step::Arc([60.0, 55.0], -1.0, 360.0), Step::To([80.0, 55.0, 5.0])]);
    }
    let mut bore = Vec::new();
    for z in [-3.0, -6.0] {
        bore.extend([Step::To([60.0, 55.0, z]), Step::To([66.0, 55.0, z]), Step::Arc([60.0, 55.0], 1.0, 360.0)]);
        bore.push(Step::To([60.0, 55.0, z]));
    }
    bore.push(Step::To([60.0, 55.0, 5.0]));
    let mut holes = Vec::new();
    for [x, y] in [[8.0, 8.0], [152.0, 8.0], [152.0, 102.0], [8.0, 102.0]] {
        holes.push(Step::To([x, y, 5.0]));
        holes.extend([-5.0, 2.0, -10.0, 2.0, -14.0, 5.0].map(|z| Step::To([x, y, z])));
    }
    // each tool's steps as one motion from where its tip starts
    let motion = |start: [f64; 3], steps: Vec<Step>| {
        let mut at = start;
        let legs = steps.into_iter().filter_map(|s| match s {
            Step::To(b) if b == at => None,
            Step::To(b) => {
                let by = Vec3::new(b[0] - at[0], b[1] - at[1], b[2] - at[2]);
                at = b;
                Some(Motion::slide(by))
            }
            Step::Arc(c, up, by) => {
                let p = Point3::new(c[0], c[1], at[2]);
                let (dx, dy, t) = (at[0] - c[0], at[1] - c[1], (up * by).to_radians());
                at = [c[0] + dx * t.cos() - dy * t.sin(), c[1] + dx * t.sin() + dy * t.cos(), at[2]];
                Some(Motion::turn(p, Vec3::new(0.0, 0.0, up), rad(0.0), deg(by)))
            }
        });
        Motion::then(legs.collect())
    };
    let drill = Loop::polygon(&[[0.0, 0.0], [4.0, 4.0], [4.0, 30.0], [0.0, 30.0]])?;
    let plate = block(&Frame::at([0.0, 0.0, -20.0]), [160.0, 110.0, 20.0])?;
    let mill = cylinder(&Frame::at([0.0, 0.0, 5.0]), 5.0, 30.0)?;
    let plate = plate.cut_swept(&mill, &motion([0.0, 0.0, 5.0], pocket))?;
    let borer = cylinder(&Frame::at([60.0, 55.0, 5.0]), 3.0, 30.0)?;
    let plate = plate.cut_swept(&borer, &motion([60.0, 55.0, 5.0], bore))?;
    let drill = revolve(&Frame::at([8.0, 8.0, 5.0]), &Profile::new(drill, vec![])?, deg(360.0))?;
    plate.cut_swept(&drill, &motion([8.0, 8.0, 5.0], holes))
}
