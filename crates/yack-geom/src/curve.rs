// yack: 3-D curves: lines, circles and ellipses in closed form, B-splines, and intersection curves.
// Adapted from springrep@a03e8a1 src/geom/curve.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/geom/curve.rs to yack-geom (M3 W3). M3 hardening: named
// constants and pieces, the nearest-point searches share their sampling and local-least test over
// iterators instead of indexing, and `extremes` asks for the spline first instead of a counted
// `panic!`; the same arithmetic in the same order.
//! 3-D curves: lines, circles and ellipses in closed form, B-splines, and intersection curves.

use std::f64::consts::{PI, SQRT_2, TAU};
use std::iter::once;

use super::{BSpline, IntersectionCurve, NurbsCurve};
use crate::{
    math::{Aabb, Frame, Point3, Transform, Vec3},
    numeric,
};

/// `origin + t·dir`, with `dir` a unit vector, so `t` is arc length.
#[derive(Clone, Debug, PartialEq)]
pub struct Line {
    pub origin: Point3,
    pub dir: Vec3,
}

/// `frame.origin + radius·(cos t·x + sin t·y)`: counter-clockwise about `frame.z`.
#[derive(Clone, Debug, PartialEq)]
pub struct Circle {
    pub frame: Frame,
    pub radius: f64,
}

/// `frame.origin + major·cos t·x + minor·sin t·y`.
#[derive(Clone, Debug, PartialEq)]
pub struct Ellipse {
    pub frame: Frame,
    pub major: f64,
    pub minor: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub enum Curve {
    Line(Line),
    Circle(Circle),
    Ellipse(Ellipse),
    BSpline(BSpline),
    Intersection(Box<IntersectionCurve>),
    /// A rational B-spline of any degree, as STEP files carry them.
    Nurbs(Box<NurbsCurve>),
}

/// The equal steps each smooth piece is sampled at when searching for a nearest point.
const SAMPLES_PER_PIECE: u32 = 16;

/// A range at least this fraction short of a period still counts as the whole period.
const WHOLE_PERIOD_SLACK: f64 = 1e-12; // tol: relative rounding of a period's length

/// The most Newton steps a nearest-point refinement takes.
const NEWTON_STEPS: usize = 50;

/// A Newton step this small, relative to the parameter, has converged.
const NEWTON_STEP_DONE: f64 = 1e-15; // tol: relative parameter rounding

/// A distance derivative this small is flat: Newton's method stops there.
const FLAT_SLOPE: f64 = 1e-300; // tol: guards a division, not a geometric tolerance

/// The intervals each smooth piece is sampled at when looking for a stationary coordinate.
const ROOT_SAMPLES_PER_PIECE: usize = 12;

/// The bisection steps that pin the nearest point of an ellipse: more than enough to reach the
/// last bit of any `f64`.
const ELLIPSE_BISECTIONS: usize = 1100;

impl Line {
    /// The line from `a` to `b`, parameterised by arc length from `a`; `None` if they coincide.
    pub fn through(a: Point3, b: Point3) -> Option<(Line, f64)> {
        let d = b - a;
        let length = d.norm();
        let dir = d.unit()?;
        Some((Line { origin: a, dir }, length))
    }
}

impl Curve {
    pub fn kind(&self) -> &'static str {
        match self {
            Curve::Line(_) => "line",
            Curve::Circle(_) => "circle",
            Curve::Ellipse(_) => "ellipse",
            Curve::BSpline(_) => "bspline",
            Curve::Intersection(_) => "intersection",
            Curve::Nurbs(_) => "nurbs",
        }
    }

    pub fn eval(&self, t: f64) -> Point3 {
        match self.spline() {
            Some(spline) => spline.eval(t),
            None => self.d2(t).0,
        }
    }

    pub fn d1(&self, t: f64) -> (Point3, Vec3) {
        match self.spline() {
            Some(spline) => {
                let [point, tangent, _] = spline.eval_ders(t, 1);
                (point, tangent)
            }
            None => {
                let (point, tangent, _) = self.d2(t);
                (point, tangent)
            }
        }
    }

    /// The spline the curve evaluates, for the B-spline and intersection curves.
    pub fn spline(&self) -> Option<&BSpline> {
        match self {
            Curve::BSpline(spline) => Some(spline),
            Curve::Intersection(curve) => Some(&curve.approx),
            _ => None,
        }
    }

    pub fn d2(&self, t: f64) -> (Point3, Vec3, Vec3) {
        match self {
            Curve::Line(line) => (line.origin + line.dir * t, line.dir, Vec3::ZERO),
            Curve::Circle(circle) => {
                let (sin, cos) = t.sin_cos();
                let f = &circle.frame;
                let r = circle.radius;
                (
                    f.origin + (f.x * cos + f.y * sin) * r,
                    (f.y * cos - f.x * sin) * r,
                    (f.x * cos + f.y * sin) * -r,
                )
            }
            Curve::Ellipse(ellipse) => {
                let (sin, cos) = t.sin_cos();
                let f = &ellipse.frame;
                let (a, b) = (ellipse.major, ellipse.minor);
                (
                    f.origin + f.x * (a * cos) + f.y * (b * sin),
                    f.y * (b * cos) - f.x * (a * sin),
                    -(f.x * (a * cos) + f.y * (b * sin)),
                )
            }
            Curve::BSpline(spline) => {
                let [point, d1, d2] = spline.eval_ders(t, 2);
                (point, d1, d2)
            }
            Curve::Intersection(curve) => {
                let [point, d1, d2] = curve.approx.eval_ders(t, 2);
                (point, d1, d2)
            }
            Curve::Nurbs(nurbs) => nurbs.d2(t),
        }
    }

    pub fn period(&self) -> Option<f64> {
        match self {
            Curve::Circle(_) | Curve::Ellipse(_) => Some(TAU),
            Curve::BSpline(spline) => spline.period(),
            Curve::Intersection(curve) => curve.approx.period(),
            Curve::Nurbs(nurbs) => nurbs.period(),
            Curve::Line(_) => None,
        }
    }

    /// The parameter range the curve is defined on; one period for a periodic curve.
    pub fn domain(&self) -> (f64, f64) {
        match self {
            Curve::Line(_) => (f64::NEG_INFINITY, f64::INFINITY),
            Curve::Circle(_) | Curve::Ellipse(_) => (0.0, TAU),
            Curve::BSpline(spline) => spline.domain(),
            Curve::Intersection(curve) => curve.approx.domain(),
            Curve::Nurbs(nurbs) => nurbs.domain(),
        }
    }

    /// Parameters strictly inside (t0, t1) where the curve may lose smoothness; quadrature and
    /// sampling split there.
    pub fn breakpoints(&self, t0: f64, t1: f64) -> Vec<f64> {
        match self {
            Curve::BSpline(spline) => spline.breakpoints(t0, t1),
            Curve::Intersection(curve) => curve.approx.breakpoints(t0, t1),
            Curve::Nurbs(nurbs) => nurbs.breakpoints(t0, t1),
            _ => Vec::new(),
        }
    }

    /// `t0`, the breakpoints between and `t1`: the ends of the curve's smooth pieces over them.
    fn piece_ends(&self, t0: f64, t1: f64) -> Vec<f64> {
        once(t0).chain(self.breakpoints(t0, t1)).chain(once(t1)).collect()
    }

    pub fn transformed(&self, t: &Transform) -> Curve {
        match self {
            Curve::Line(line) => {
                Curve::Line(Line { origin: t.point(line.origin), dir: t.vector(line.dir) })
            }
            Curve::Circle(circle) => {
                Curve::Circle(Circle { frame: circle.frame.transformed(t), radius: circle.radius })
            }
            Curve::Ellipse(ellipse) => {
                Curve::Ellipse(Ellipse { frame: ellipse.frame.transformed(t), ..*ellipse })
            }
            Curve::BSpline(spline) => Curve::BSpline(spline.transformed(t)),
            Curve::Intersection(curve) => Curve::Intersection(Box::new(curve.transformed(t))),
            Curve::Nurbs(nurbs) => Curve::Nurbs(Box::new(nurbs.transformed(t))),
        }
    }

    /// The parameter in [t0, t1] of the point of the curve nearest `p`.
    pub fn project(&self, p: Point3, t0: f64, t1: f64) -> f64 {
        match self {
            Curve::Line(line) => (p - line.origin).dot(line.dir).clamp(t0, t1),
            Curve::Circle(circle) => {
                let q = circle.frame.to_local(p);
                self.angle_in_range(q.y.atan2(q.x), p, t0, t1)
            }
            Curve::Ellipse(ellipse) => {
                let q = ellipse.frame.to_local(p);
                let t = ellipse_nearest(ellipse.major, ellipse.minor, q.x, q.y);
                let t = t0 + (t - t0).rem_euclid(TAU);
                if t <= t1 {
                    return t;
                }
                // The nearest point lies outside the range: the nearest within it is an end or a
                // second local least, which the sampled search finds.
                let sampled = self.project_numeric(p, t0, t1);
                [t0, t1, sampled]
                    .into_iter()
                    .min_by(|&x, &y| self.eval(x).dist(p).total_cmp(&self.eval(y).dist(p)))
                    .unwrap_or(sampled)
            }
            Curve::Nurbs(nurbs) => {
                let (start, end) = nurbs.domain();
                if nurbs.period().is_none() && (t0 < start || t1 > end) {
                    self.project_numeric(p, t0, t1)
                } else {
                    self.project_pieces(p, t0, t1, |t| nurbs.span_box(t))
                }
            }
            _ => match self.spline() {
                Some(spline) => spline.nearest(p, None, t0, t1),
                None => self.project_numeric(p, t0, t1),
            },
        }
    }

    /// The angle `angle` moved by whole turns into [t0, t1], or else the nearer end of the range.
    fn angle_in_range(&self, angle: f64, p: Point3, t0: f64, t1: f64) -> f64 {
        let angle = t0 + (angle - t0).rem_euclid(TAU);
        if angle <= t1 {
            return angle;
        }
        let to_start = self.eval(t0).dist(p);
        let to_end = self.eval(t1).dist(p);
        if to_start <= to_end { t0 } else { t1 }
    }

    /// The period, if [t0, t1] spans a whole one: a nearest-point search then wraps round it.
    fn period_spanned(&self, t0: f64, t1: f64) -> Option<f64> {
        self.period().filter(|&period| t1 - t0 >= period * (1.0 - WHOLE_PERIOD_SLACK))
    }

    /// `(distance to p, t)` at [`SAMPLES_PER_PIECE`] equal steps over [a, b], both ends included.
    fn piece_samples(&self, p: Point3, a: f64, b: f64) -> impl Iterator<Item = (f64, f64)> + '_ {
        (0..=SAMPLES_PER_PIECE).map(move |k| {
            let t = a + (b - a) * f64::from(k) / f64::from(SAMPLES_PER_PIECE);
            (self.eval(t).dist(p), t)
        })
    }

    /// The nearest point by samples, each local least of them refined by Newton's method. Over a
    /// whole period of a closed curve the parameter wraps round instead of stopping at the range's
    /// ends, which are one point: the nearest point can lie just across them from the best sample.
    fn project_numeric(&self, p: Point3, t0: f64, t1: f64) -> f64 {
        let wrap = self.period_spanned(t0, t1);
        let ends = self.piece_ends(t0, t1);
        let samples: Vec<(f64, f64)> =
            consecutive(&ends).flat_map(|(a, b)| self.piece_samples(p, a, b)).collect();
        let mut best = samples
            .iter()
            .copied()
            .fold((f64::INFINITY, t0), |best, s| if s.0 < best.0 { s } else { best });
        for start in local_leasts(&samples) {
            let t = self.refine_nearest(p, start, t0, t1, wrap);
            let distance = self.eval(t).dist(p);
            if distance < best.0 {
                best = (distance, t);
            }
        }
        best.1
    }

    /// [`Curve::project_numeric`] searching only the pieces between breakpoints whose box, from
    /// `hull`, could hold a point nearer than the best found, nearest box first. Each piece is
    /// sampled as there, and a sample is a local least as there, its neighbour across a piece's end
    /// being the same point.
    fn project_pieces(&self, p: Point3, t0: f64, t1: f64, hull: impl Fn(f64) -> Aabb) -> f64 {
        let wrap = self.period_spanned(t0, t1);
        let ends = self.piece_ends(t0, t1);
        let mut pieces: Vec<(f64, f64, f64)> = consecutive(&ends)
            .map(|(a, b)| (box_distance(&hull(0.5 * (a + b)), p), a, b))
            .collect();
        pieces.sort_by(|x, y| x.0.total_cmp(&y.0));
        let mut best = (f64::INFINITY, t0);
        let mut samples = [(0.0, 0.0); SAMPLES_PER_PIECE as usize + 1];
        for (box_gap, a, b) in pieces {
            if box_gap > best.0 {
                break;
            }
            for (slot, sample) in samples.iter_mut().zip(self.piece_samples(p, a, b)) {
                *slot = sample;
                if sample.0 < best.0 {
                    best = sample;
                }
            }
            for start in local_leasts(&samples) {
                let t = self.refine_nearest(p, start, t0, t1, wrap);
                let distance = self.eval(t).dist(p);
                if distance < best.0 {
                    best = (distance, t);
                }
            }
        }
        best.1
    }

    /// Newton's method on the squared distance to `p` from `t`, held to [t0, t1] or wrapped
    /// round `wrap`.
    fn refine_nearest(&self, p: Point3, mut t: f64, t0: f64, t1: f64, wrap: Option<f64>) -> f64 {
        for _ in 0..NEWTON_STEPS {
            let (point, d1, d2) = self.d2(t);
            let r = point - p;
            let slope = r.dot(d1);
            let curvature = d1.norm2() + r.dot(d2);
            if curvature.abs() < FLAT_SLOPE {
                break;
            }
            let step = slope / curvature.max(d1.norm2() * 0.1);
            let next = match wrap {
                Some(period) => t0 + (t - step - t0).rem_euclid(period),
                None => (t - step).clamp(t0, t1),
            };
            let done = step.abs() <= NEWTON_STEP_DONE * (1.0 + t.abs()) || next == t;
            t = next;
            if done {
                break;
            }
        }
        t
    }

    /// The tight box of the curve over [t0, t1].
    pub fn bbox(&self, t0: f64, t1: f64) -> Aabb {
        if let Some(spline) = self.spline() {
            return spline.bbox(t0, t1);
        }
        let mut bb = Aabb::EMPTY;
        bb.add(self.eval(t0));
        bb.add(self.eval(t1));
        for axis in [Vec3::X, Vec3::Y, Vec3::Z] {
            for t in self.extremes(axis, t0, t1) {
                bb.add(self.eval(t));
            }
        }
        bb
    }

    /// Parameters strictly inside (t0, t1) where the curve's coordinate along `dir` is stationary.
    pub fn extremes(&self, dir: Vec3, t0: f64, t1: f64) -> Vec<f64> {
        if let Some(spline) = self.spline() {
            return spline.extremes(dir, t0, t1);
        }
        let inside = |t: &f64| *t > t0 && *t < t1;
        // A conic's coordinate is stationary at `base` and every half turn from it.
        let half_turns_from = |base: f64| -> Vec<f64> {
            let first = ((t0 - base) / PI).floor() as i64;
            let last = ((t1 - base) / PI).ceil() as i64;
            (first..=last).map(|k| base + k as f64 * PI).filter(inside).collect()
        };
        match self {
            Curve::Line(_) => Vec::new(),
            Curve::Circle(circle) => {
                half_turns_from(dir.dot(circle.frame.y).atan2(dir.dot(circle.frame.x)))
            }
            Curve::Ellipse(ellipse) => half_turns_from(
                (ellipse.minor * dir.dot(ellipse.frame.y))
                    .atan2(ellipse.major * dir.dot(ellipse.frame.x)),
            ),
            _ => {
                let ends = self.piece_ends(t0, t1);
                let mut along = |t: f64| self.d1(t).1.dot(dir);
                consecutive(&ends)
                    .flat_map(|(a, b)| numeric::roots(&mut along, a, b, ROOT_SAMPLES_PER_PIECE))
                    .filter(inside)
                    .collect()
            }
        }
    }
}

/// Each value paired with the next.
fn consecutive(values: &[f64]) -> impl Iterator<Item = (f64, f64)> + '_ {
    values.iter().copied().zip(values.iter().copied().skip(1))
}

/// The parameters of the `(distance, t)` samples no farther than either neighbour: the local
/// leasts of the distance, the ends having no neighbour beyond them.
fn local_leasts(samples: &[(f64, f64)]) -> impl Iterator<Item = f64> + '_ {
    let distances = samples.iter().map(|&(distance, _)| distance);
    let before = once(f64::INFINITY).chain(distances.clone());
    let after = distances.skip(1).chain(once(f64::INFINITY));
    before.zip(samples).zip(after).filter_map(|((before, &(distance, t)), after)| {
        let farther = distance > before || distance > after;
        (!farther).then_some(t)
    })
}

/// The distance from `p` to the nearest point of the box `bb`; zero inside it.
fn box_distance(bb: &Aabb, p: Point3) -> f64 {
    Vec3::new(
        (bb.min.x - p.x).max(p.x - bb.max.x).max(0.0),
        (bb.min.y - p.y).max(p.y - bb.max.y).max(0.0),
        (bb.min.z - p.z).max(p.z - bb.max.z).max(0.0),
    )
    .norm()
}

/// The parameter `t` of the point `(a cos t, b sin t)` nearest `(x, y)`, solved rather than
/// searched for.
///
/// The nearest point `(u, v)` has `(x, y)` on its normal: `x − u = λ·u/a²` and `y − v = λ·v/b²`
/// for one `λ`, so `u = a²x/(λ + a²)` and `v = b²y/(λ + b²)`. Putting those on the ellipse leaves
/// one equation, `F(λ) = (a·x/(λ + a²))² + (b·y/(λ + b²))² − 1 = 0`. With both of `x`, `y`
/// positive `F` falls steadily for `λ` above `−min(a, b)²`, so it has one root there and
/// bisection pins it to the last bit. The other quadrants follow by symmetry. On an axis the
/// equation loses a term, and the answer is the vertex on that axis, or, on the longer axis
/// inside the centres of curvature of its vertices, the point whose normal passes through.
fn ellipse_nearest(a: f64, b: f64, x: f64, y: f64) -> f64 {
    let (px, py) = (x.abs(), y.abs());
    let (a2, b2) = (a * a, b * b);
    let (u, v) = if px == 0.0 && py == 0.0 {
        // The centre is nearest the ends of the shorter axis.
        if a <= b { (a, 0.0) } else { (0.0, b) }
    } else if py == 0.0 {
        // On the x axis: the normal from `(u, v)` meets it at `u·(a² − b²)/a²`.
        let foot = a * px / (a2 - b2);
        if a > b && foot < 1.0 {
            (a * foot, b * (1.0 - foot * foot).max(0.0).sqrt())
        } else {
            (a, 0.0)
        }
    } else if px == 0.0 {
        let foot = b * py / (b2 - a2);
        if b > a && foot < 1.0 {
            (a * (1.0 - foot * foot).max(0.0).sqrt(), b * foot)
        } else {
            (0.0, b)
        }
    } else {
        let f = |l: f64| (a * px / (l + a2)).powi(2) + (b * py / (l + b2)).powi(2) - 1.0;
        // F is positive just above −min(a, b)², and negative once λ passes the larger of the
        // two terms' own roots: each term is then below a half.
        let mut lo = -a2.min(b2);
        let mut hi = (SQRT_2 * a * px - a2).max(SQRT_2 * b * py - b2).max(lo);
        let mut l = hi;
        for _ in 0..ELLIPSE_BISECTIONS {
            l = 0.5 * (lo + hi);
            if l == lo || l == hi {
                break;
            }
            let at = f(l);
            if at > 0.0 {
                lo = l;
            } else if at < 0.0 {
                hi = l;
            } else {
                break;
            }
        }
        (a2 * px / (l + a2), b2 * py / (l + b2))
    };
    (v.copysign(y) / b).atan2(u.copysign(x) / a)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn circle() -> Curve {
        let frame = Frame::new(Vec3::new(1.0, 2.0, 3.0), Vec3::Z, Vec3::X).unwrap();
        Curve::Circle(Circle { frame, radius: 5.0 })
    }

    #[test]
    fn circle_derivatives_match_differences() {
        let c = circle();
        let (_, d1, d2) = c.d2(0.7);
        let h = 1e-6;
        let fd = (c.eval(0.7 + h) - c.eval(0.7 - h)) / (2.0 * h);
        let fd2 = (c.eval(0.7 + h) - c.eval(0.7) * 2.0 + c.eval(0.7 - h)) / (h * h);
        assert!(d1.dist(fd) < 1e-8);
        assert!(d2.dist(fd2) < 1e-3);
    }

    #[test]
    fn circle_projection_respects_its_range() {
        let c = circle();
        let p = Vec3::new(1.0, 12.0, 3.0);
        assert!((c.project(p, 0.0, TAU) - PI / 2.0).abs() < 1e-15);
        assert!((c.project(p, 4.0, 7.0) - 7.0).abs() < 1e-15);
        assert!((c.project(p, 6.0, 8.0) - (PI / 2.0 + TAU)).abs() < 1e-14);
    }

    #[test]
    fn ellipse_projection_finds_a_nearest_point_just_across_its_start() {
        // The nearest point is at t = -0.08, a period from the range's end: the best of the
        // samples is the start, and a step clamped there stops short of it.
        let e = Curve::Ellipse(Ellipse { frame: Frame::WORLD, major: 5.0, minor: 3.0 });
        let want = TAU - 0.08;
        let p = e.eval(want);
        let t = e.project(p, 0.0, TAU);
        assert!(e.eval(t).dist(p) < 1e-12, "{t}");
        assert!((t - want).abs() < 1e-9, "{t}");
    }

    #[test]
    fn ellipse_projection_is_exact_beside_a_sharp_tip() {
        // A long thin ellipse, its two sides a hair apart near the tip: every point on it
        // projects to itself, wherever the samples fall.
        let e = Curve::Ellipse(Ellipse { frame: Frame::WORLD, major: 20.0, minor: 0.3 });
        for k in 0..400 {
            let t = TAU * k as f64 / 400.0 + 0.003;
            let p = e.eval(t);
            let s = e.project(p, 0.0, TAU);
            assert!(e.eval(s).dist(p) < 1e-12, "{t} → {s}");
            // Off the plane and off the curve the nearest point is unchanged by the height.
            let q = p + Vec3::Z * 3.0;
            assert!(e.eval(e.project(q, 0.0, TAU)).dist(p) < 1e-9, "{t}");
        }
        // Inside, near the major axis, the nearest points are a symmetric pair either side.
        let inside = Vec3::new(10.0, 0.0, 0.0);
        let s = e.project(inside, 0.0, TAU);
        let best = (0..100_000)
            .map(|k| e.eval(TAU * k as f64 / 100_000.0).dist(inside))
            .fold(f64::INFINITY, f64::min);
        assert!(e.eval(s).dist(inside) <= best + 1e-12);
    }

    #[test]
    fn ellipse_projection_and_extremes() {
        let frame = Frame::WORLD;
        let e = Curve::Ellipse(Ellipse { frame, major: 30.0, minor: 20.0 });
        let t = e.project(Vec3::new(40.0, 1.0, 0.0), 0.0, TAU);
        assert!(t.abs() < 0.1);
        let ext = e.extremes(Vec3::X, 0.0, TAU);
        assert_eq!(ext.len(), 1);
        assert!((ext[0] - PI).abs() < 1e-15);
    }

    /// A rational cubic winding three turns round z on radius 5, its weights uneven: rising 1 a
    /// turn and open, or bobbing and closed, its ends meeting.
    fn coil(closed: bool) -> Curve {
        let n = 60;
        let mut ctrl: Vec<Point3> = (0..n)
            .map(|i| {
                let a = TAU * 3.0 * i as f64 / (n - 1) as f64;
                let z = if closed { (a / 3.0).sin() } else { a / TAU };
                Vec3::new(5.0 * a.cos(), 5.0 * a.sin(), z)
            })
            .collect();
        if closed {
            ctrl[n - 1] = ctrl[0];
        }
        let knots: Vec<f64> =
            (0..n + 4).map(|i| (i as f64 - 3.0).clamp(0.0, (n - 3) as f64)).collect();
        let weights = (0..n).map(|i| 1.0 + 0.3 * (i % 3) as f64).collect();
        Curve::Nurbs(Box::new(NurbsCurve::new(3, knots, ctrl, Some(weights)).unwrap()))
    }

    #[test]
    fn projecting_by_pieces_finds_the_points_sampling_everywhere_finds() {
        // Points on a grid through and round the coil, over its whole range and over parts of it:
        // searching only the pieces whose boxes could hold a nearer point reaches the same least
        // distance as sampling every piece.
        for closed in [false, true] {
            let c = coil(closed);
            assert_eq!(c.period().is_some(), closed);
            let (a, b) = c.domain();
            let mut ranges = vec![(a, b), (a + 3.3, b - 10.1), (a + 20.0, a + 21.5)];
            if closed {
                // Across the closed coil's ends, where its parameter wraps round.
                ranges.push((b - 4.0, b + 3.0));
            }
            for (t0, t1) in ranges {
                for i in 0..9 {
                    for j in 0..9 {
                        for k in 0..5 {
                            let p = Vec3::new(
                                -7.0 + 1.75 * i as f64,
                                -7.0 + 1.75 * j as f64,
                                -0.5 + k as f64,
                            );
                            let full = c.eval(c.project_numeric(p, t0, t1)).dist(p);
                            let s = c.project(p, t0, t1);
                            assert!((t0..=t1).contains(&s));
                            let got = c.eval(s).dist(p);
                            assert!((got - full).abs() <= 1e-12, "{p:?}: {got} {full}");
                        }
                    }
                }
            }
        }
    }
}
