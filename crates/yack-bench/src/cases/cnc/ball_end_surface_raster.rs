// yack: brep-bench case cnc/ball-end-surface-raster
//! A free-form surface finished by a ball-end mill in 41 three-axis raster passes.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    // The program as data: the tip's points, the surface sampled along 41 zigzag passes; every move is a slide.
    let pi = std::f64::consts::PI;
    let f = |x: f64, y: f64| -5.0 + 4.0 * (pi * x / 50.0).sin() * (pi * y / 40.0).sin();
    let mut points = vec![Point3::new(-5.0, 0.0, 5.0)];
    for j in 0..=40 {
        let y = 2.0 * f64::from(j);
        let along: Vec<i32> = if j % 2 == 0 { (0..=22).collect() } else { (0..=22).rev().collect() };
        points.extend(along.into_iter().map(|i| -5.0 + 5.0 * f64::from(i)).map(|x| Point3::new(x, y, f(x, y))));
    }
    points.push(Point3::new(-5.0, 80.0, 5.0));
    // the ball-end mill, radius 3, its tip at (−5, 0, 5)
    let mill = cutter(&Frame::at([-5.0, 0.0, 5.0]), Cutter::Ball, 3.0, 30.0)?;
    block(&Frame::at([0.0, 0.0, -20.0]), [100.0, 80.0, 20.0])?.cut_swept(&mill, &Motion::through(&points)?)
}
