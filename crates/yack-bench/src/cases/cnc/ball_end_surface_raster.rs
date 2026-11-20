// yack: brep-bench case cnc/ball-end-surface-raster
//! A free-form surface finished by a ball-end mill in 41 three-axis raster passes.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    // The program as data: the tip's points, the surface sampled along 41 zigzag passes; every move is a slide.
    let pi = std::f64::consts::PI;
    let f = |x: f64, y: f64| -5.0 + 4.0 * (pi * x / 50.0).sin() * (pi * y / 40.0).sin();
    let mut points = vec![[-5.0, 0.0, 5.0]];
    for j in 0..=40 {
        let y = 2.0 * f64::from(j);
        let along: Vec<i32> = if j % 2 == 0 { (0..=22).collect() } else { (0..=22).rev().collect() };
        points.extend(along.into_iter().map(|i| [-5.0 + 5.0 * f64::from(i), y, f(-5.0 + 5.0 * f64::from(i), y)]));
    }
    points.push([-5.0, 80.0, 5.0]);
    let legs = points.windows(2).filter_map(|w| match w {
        [a, b] => Some(Motion::slide(Vec3::new(b[0] - a[0], b[1] - a[1], b[2] - a[2]))),
        _ => None,
    });
    // the ball-end mill, radius 3, its tip at (−5, 0, 5)
    let side = Loop::starting_at([0.0, 0.0]).arc_ccw_to([3.0, 3.0], [0.0, 3.0]);
    let side = side.line_to([3.0, 30.0]).line_to([0.0, 30.0]);
    let mill = revolve(&Frame::at([-5.0, 0.0, 5.0]), &Profile::new(side.close()?, vec![])?, deg(360.0))?;
    block(&Frame::at([0.0, 0.0, -20.0]), [100.0, 80.0, 20.0])?.cut_swept(&mill, &Motion::then(legs.collect()))
}
