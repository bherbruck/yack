// yack: brep-bench case swept/spline-bar-screwed-through-rod
//! A rod threaded by a bar whose section is a spline.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let c = [[3.5, 0.0], [4.5, -1.5], [8.0, -1.5], [12.0, -1.0], [12.0, 1.0], [8.0, 1.5], [4.5, 1.5]].map(Point2::from);
    let ctrl = c.iter().chain(c.iter().take(3)).copied().collect();
    let spline = Segment::PeriodicSpline { degree: 3, knots: (0..14).map(f64::from).collect(), ctrl };
    // The section drawn in y = 0 and carried along y, as placed turned -90° about z and lowered 1.5.
    let start = Frame::new([8.0, 0.0, -1.5], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0], [-1.0, 0.0, 0.0])?;
    let bar = extrude(&start, &Profile::new(Loop::new(vec![spline])?, vec![])?, 16.0)?;
    let up = Vec3::new(0.0, 0.0, 1.0);
    let screw = Motion::screw(Point3::ORIGIN, up, rad(0.0), deg(1440.0), Lead::per_turn(6.0));
    cylinder(&Frame::WORLD, 5.0, 20.0)?.cut_swept(&bar, &screw)
}
