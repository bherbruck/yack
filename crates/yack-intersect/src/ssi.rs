// yack: Surface–surface intersection. Planes against the analytic surfaces, coaxial surfaces of
// Adapted from springrep@a03e8a1 src/boolean/ssi.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from boolean/ssi.rs to intersect/ssi.rs (M2.1, T2); paths only; then to yack-intersect (M3 W4), paths and visibility only; `.expect`/`.unwrap` as counted `panic!`s (gate 9).
// M3 hardening: split into submodules by pair (plane, coaxial, cylinders), a closed form that
// cannot build its frame declining to the tracer instead of panicking, the region's corners and
// axes shared, checked access instead of indexing; the same arithmetic in the same order.
//! Surface–surface intersection. Planes against the analytic surfaces, coaxial surfaces of
//! revolution, parallel cylinders and equal crossing cylinders have closed forms; every other pair
//! is traced by [`super::march`].
//!
//! Curves come back over a range that covers the region asked about; the caller cuts them at the
//! faces' boundaries. A curve along which the surfaces are tangent is marked, and isolated points
//! where they are tangent (where two branches cross) come back as `points`.

mod coaxial;
mod conic;
mod cylinders;
mod plane;
mod sweep;
#[cfg(test)]
mod tests;

use std::f64::consts::TAU;

use self::coaxial::{Meridian, circles_about, coaxial, meridian, meridian_points};
use super::march::{self, SsiCurve};
use crate::{
    Result,
    geom::{Circle, Cone, Curve, Line, Surface},
    math::{Aabb, Frame, Point3, Vec2, Vec3},
    tol,
};

#[derive(Clone, Debug)]
pub enum Ssi {
    /// The surfaces are the same point set.
    Coincident,
    Curves {
        curves: Vec<SsiCurve>,
        points: Vec<Point3>,
    },
}

impl Ssi {
    /// No meeting at all.
    fn none() -> Ssi {
        Ssi::Curves { curves: Vec::new(), points: Vec::new() }
    }

    /// The meeting along `curves` alone.
    fn curves(curves: Vec<SsiCurve>) -> Ssi {
        Ssi::Curves { curves, points: Vec::new() }
    }
}

/// A cone whose half angle's sine is no more than this is a cylinder, with no apex.
const NO_APEX: f64 = 1e-12; // tol: a sine that vanishes

/// Below this a direction's part along an axis is zero.
const UNDERFLOW: f64 = 1e-300; // tol: an underflow guard

/// The samples along each parameter of the window [`within`] tests a surface over.
const WINDOW_SAMPLES: usize = 6;

/// Whether two directions are parallel within the angular tolerance.
fn parallel(a: Vec3, b: Vec3) -> bool {
    a.cross(b).norm() <= tol::ANGULAR
}

/// The distance from `p` to the line through `o` along the unit `d`.
fn line_distance(p: Point3, o: Point3, d: Vec3) -> f64 {
    let w = p - o;
    (w - d * w.dot(d)).norm()
}

/// An axis of symmetry: a surface of revolution's, or any line through a sphere's centre.
#[derive(Clone, Copy, Debug)]
struct Axis {
    origin: Point3,
    dir: Vec3,
}

impl Axis {
    /// A frame's `z` axis.
    fn of(frame: &Frame) -> Axis {
        Axis { origin: frame.origin, dir: frame.z }
    }
}

/// The axis of a cylinder, cone, torus or sphere.
fn axis_of(s: &Surface) -> Option<Axis> {
    match s {
        Surface::Cylinder(c) => Some(Axis::of(&c.frame)),
        Surface::Cone(c) => Some(Axis::of(&c.frame)),
        Surface::Torus(t) => Some(Axis::of(&t.frame)),
        Surface::Sphere(sp) => Some(Axis::of(&sp.frame)),
        _ => None,
    }
}

/// Whether two axes are one line, within the tolerances.
fn same_axis(a: Axis, b: Axis) -> bool {
    parallel(a.dir, b.dir) && line_distance(b.origin, a.origin, a.dir) <= tol::LINEAR
}

/// The eight corners of a box.
fn corners(region: &Aabb) -> impl Iterator<Item = Point3> + '_ {
    (0..8).map(|k| {
        let pick = |bit: i32, lo: f64, hi: f64| if k & bit == 0 { lo } else { hi };
        Point3::new(
            pick(1, region.min.x, region.max.x),
            pick(2, region.min.y, region.max.y),
            pick(4, region.min.z, region.max.z),
        )
    })
}

/// Whether two surfaces are the same point set.
pub fn same_domain(a: &Surface, b: &Surface) -> bool {
    let close = |x: f64, y: f64| (x - y).abs() <= tol::LINEAR;
    match (a, b) {
        (Surface::Plane(p), Surface::Plane(q)) => {
            parallel(p.frame.z, q.frame.z)
                && (q.frame.origin - p.frame.origin).dot(p.frame.z).abs() <= tol::LINEAR
        }
        (Surface::Cylinder(p), Surface::Cylinder(q)) => {
            close(p.radius, q.radius) && same_axis(Axis::of(&p.frame), Axis::of(&q.frame))
        }
        (Surface::Sphere(p), Surface::Sphere(q)) => {
            close(p.radius, q.radius) && p.frame.origin.dist(q.frame.origin) <= tol::LINEAR
        }
        (Surface::Torus(p), Surface::Torus(q)) => {
            close(p.major, q.major)
                && close(p.minor, q.minor)
                && p.frame.origin.dist(q.frame.origin) <= tol::LINEAR
                && parallel(p.frame.z, q.frame.z)
        }
        (Surface::Cone(p), Surface::Cone(q)) => same_cone(p, q),
        _ => a == b || (within(a, b) && within(b, a)),
    }
}

/// Whether two cones are one point set: one axis, one apex and one slope.
fn same_cone(p: &Cone, q: &Cone) -> bool {
    if !same_axis(Axis::of(&p.frame), Axis::of(&q.frame)) {
        return false;
    }
    let apex = |c: &Cone| {
        let (sa, ca) = c.half_angle.sin_cos();
        (sa.abs() > NO_APEX).then(|| c.frame.origin + c.frame.z * (-c.radius / sa * ca))
    };
    let slope = |c: &Cone, z: Vec3| {
        let t = c.half_angle.tan();
        if c.frame.z.dot(z) > 0.0 { t } else { -t }
    };
    match (apex(p), apex(q)) {
        (Some(x), Some(y)) => {
            x.dist(y) <= tol::LINEAR
                && (slope(p, p.frame.z).abs() - slope(q, p.frame.z).abs()).abs() <= tol::ANGULAR
        }
        _ => false,
    }
}

/// A piece of a swept or spline surface's parameters that stands for the whole of it: a spline's
/// own domain, a turn of a revolution, a ruled surface's unit square, and of an extrusion its
/// profile carried a way along. `None` for a surface with a closed form, and for one whose profile
/// is a line without ends.
fn sample_window(s: &Surface) -> Option<(Vec2, Vec2)> {
    let ends = |c: &Curve| {
        let (lo, hi) = c.domain();
        (lo.is_finite() && hi.is_finite()).then_some((lo, hi))
    };
    Some(match s {
        Surface::Extrusion(e) => {
            let (lo, hi) = ends(&e.curve)?;
            let reach = e.curve.bbox(lo, hi).diagonal().max(1.0);
            (Vec2::new(lo, 0.0), Vec2::new(hi, reach))
        }
        Surface::Revolution(r) => {
            let (lo, hi) = ends(&r.curve)?;
            (Vec2::new(0.0, lo), Vec2::new(TAU, hi))
        }
        Surface::Ruled(_) => (Vec2::ZERO, Vec2::new(1.0, 1.0)),
        Surface::Nurbs(ns) => {
            let ((u0, u1), (v0, v1)) = ns.domain();
            (Vec2::new(u0, v0), Vec2::new(u1, v1))
        }
        _ => return None,
    })
}

/// Whether surface `a`, over the piece of its parameters that stands for it, lies on `b` with the
/// same normal line: the test [`same_domain_faces`] makes over two faces' own windows. Two swept
/// surfaces on one point set are seldom equal part for part: the same profile slid along its
/// extrusion, or turned about its axis, draws the same surface from another start.
fn within(a: &Surface, b: &Surface) -> bool {
    let Some((lo, hi)) = sample_window(a) else {
        return false;
    };
    let n = WINDOW_SAMPLES;
    (0..=n).all(|i| {
        (0..=n).all(|j| {
            let u = lo.x + (hi.x - lo.x) * i as f64 / n as f64;
            let v = lo.y + (hi.y - lo.y) * j as f64 / n as f64;
            let p = a.eval(u, v);
            let q = b.project(p);
            b.eval(q.x, q.y).dist(p) <= tol::LINEAR
                && a.normal(u, v).cross(b.normal(q.x, q.y)).norm() <= tol::TANGENT
        })
    })
}

/// The part of the line `o + t·d` inside `region`, as a parameter range.
pub fn clip_line(o: Point3, d: Vec3, region: &Aabb) -> Option<(f64, f64)> {
    let (mut lo, mut hi) = (f64::NEG_INFINITY, f64::INFINITY);
    let (min, max) = (region.min.to_array(), region.max.to_array());
    for (((ok, dk), mn), mx) in o.to_array().into_iter().zip(d.to_array()).zip(min).zip(max) {
        if dk.abs() < UNDERFLOW {
            if ok < mn || ok > mx {
                return None;
            }
            continue;
        }
        let (a, b) = ((mn - ok) / dk, (mx - ok) / dk);
        lo = lo.max(a.min(b));
        hi = hi.min(a.max(b));
    }
    (hi > lo).then_some((lo, hi))
}

/// The line through `o` along `d`, over its part inside `region`.
fn line_curve(o: Point3, d: Vec3, region: &Aabb, tangent: bool) -> Option<SsiCurve> {
    let range = clip_line(o, d, region)?;
    Some(SsiCurve { curve: Curve::Line(Line { origin: o, dir: d }), range, closed: false, tangent })
}

/// The whole circle of `radius` in `frame`.
fn circle_curve(frame: Frame, radius: f64, tangent: bool) -> SsiCurve {
    SsiCurve {
        curve: Curve::Circle(Circle { frame, radius }),
        range: (0.0, TAU),
        closed: true,
        tangent,
    }
}

/// The frame at `origin` about the unit `z`, its `x` toward `x_hint` where that has a part square
/// to `z`; `None` only for a `z` of no length.
#[allow(clippy::panic)]
fn frame_at(origin: Point3, z: Vec3, x_hint: Vec3) -> Option<Frame> {
    Some(
        Frame::new(origin, z, x_hint)
            .or_else(|_| Frame::from_axis(origin, z))
            .unwrap_or_else(|e| panic!("axis is a unit vector: {e:?}")),
    )
}

/// The intersection of `a` and `b` within `region`. `seeds` gives points on both surfaces where
/// tracing may start, for the pairs with no closed form.
pub fn intersect(
    a: &Surface,
    b: &Surface,
    region: &Aabb,
    seeds: &mut dyn FnMut() -> Vec<Point3>,
) -> Result<Ssi> {
    if same_domain(a, b) {
        return Ok(Ssi::Coincident);
    }
    if let Some(r) = closed_form(a, b, region) {
        return Ok(r);
    }
    if let Some(r) = closed_form(b, a, region) {
        return Ok(r);
    }
    let (curves, points) = march::trace(a, b, region, &seeds())?;
    Ok(Ssi::Curves { curves, points })
}

/// Whether moving along `dir` leaves `s` unchanged.
fn translation_invariant(s: &Surface, dir: Vec3) -> bool {
    match s {
        Surface::Plane(p) => p.frame.z.dot(dir).abs() <= tol::ANGULAR,
        Surface::Cylinder(c) => parallel(c.frame.z, dir),
        Surface::Extrusion(e) => parallel(e.dir, dir),
        _ => false,
    }
}

/// Whether turning about `axis` leaves `s` unchanged.
fn rotation_invariant(s: &Surface, axis: Axis) -> bool {
    match s {
        Surface::Plane(p) => parallel(p.frame.z, axis.dir),
        Surface::Sphere(sp) => line_distance(sp.frame.origin, axis.origin, axis.dir) <= tol::LINEAR,
        Surface::Cylinder(_) | Surface::Cone(_) | Surface::Torus(_) => {
            axis_of(s).is_some_and(|x| same_axis(x, axis))
        }
        Surface::Revolution(r) => same_axis(Axis { origin: r.origin, dir: r.axis }, axis),
        _ => false,
    }
}

/// Whether the surfaces are tangent at `p`, a point of both.
fn tangent_at(a: &Surface, b: &Surface, p: Point3) -> bool {
    let (ua, ub) = (a.project(p), b.project(p));
    a.normal(ua.x, ua.y).cross(b.normal(ub.x, ub.y)).norm() <= tol::TANGENT
}

/// An extrusion or revolution against a surface that shares its symmetry: the profile curve's
/// meetings with the other surface, swept the same way, are lines or circles; a plane across an
/// extrusion cuts it in a copy of its curve.
fn swept(a: &Surface, b: &Surface, region: &Aabb) -> Option<Ssi> {
    match a {
        Surface::Extrusion(e) => {
            if let Surface::Plane(p) = b
                && parallel(p.frame.z, e.dir)
            {
                let (d0, _) = e.curve.domain();
                let on = e.curve.eval(d0);
                let n = p.frame.z;
                let shift = e.dir * ((p.frame.origin - on).dot(n) / e.dir.dot(n));
                let curve = e.curve.transformed(&crate::math::Transform::translation(shift));
                let range = curve.domain();
                let closed = curve.period().is_some();
                return Some(Ssi::curves(vec![SsiCurve { curve, range, closed, tangent: false }]));
            }
            if !translation_invariant(b, e.dir) {
                return None;
            }
            let hits = super::csi::curve_surface(&e.curve, e.curve.domain(), b);
            if hits.on_surface {
                return Some(Ssi::Coincident);
            }
            let curves = hits
                .ts
                .into_iter()
                .map(|t| e.curve.eval(t))
                .filter_map(|q| line_curve(q, e.dir, region, tangent_at(a, b, q)))
                .collect();
            Some(Ssi::curves(curves))
        }
        Surface::Revolution(r) => {
            let axis = Axis { origin: r.origin, dir: r.axis };
            if !rotation_invariant(b, axis) {
                return None;
            }
            let hits = super::csi::curve_surface(&r.curve, r.curve.domain(), b);
            if hits.on_surface {
                return Some(Ssi::Coincident);
            }
            let pts = hits
                .ts
                .into_iter()
                .map(|t| {
                    let q = r.curve.eval(t);
                    let z = (q - axis.origin).dot(axis.dir);
                    (Vec2::new(line_distance(q, axis.origin, axis.dir), z), tangent_at(a, b, q))
                })
                .collect();
            circles_about(axis, axis.dir.any_perpendicular(), pts)
        }
        _ => None,
    }
}

/// A plane square to `axis` against a surface of revolution `s` about it, as a degenerate surface
/// of revolution: their meridians' meetings, as circles about the axis.
fn plane_across_axis(plane: &Frame, s: &Surface, axis: Axis) -> Option<Ssi> {
    let h = (plane.origin - axis.origin).dot(axis.dir);
    let plane_meridian = Meridian::Line { p: Vec2::new(0.0, h), d: Vec2::new(1.0, 0.0) };
    let pts = meridian_points(plane_meridian, meridian(s, axis)?)?;
    circles_about(axis, axis.dir.any_perpendicular(), pts)
}

/// The closed form for `a` against `b`, taken that way round, where one applies.
fn closed_form(a: &Surface, b: &Surface, region: &Aabb) -> Option<Ssi> {
    if let Some(r) = swept(a, b, region) {
        return Some(r);
    }
    match (a, b) {
        (Surface::Plane(p), Surface::Plane(q)) => {
            return plane::plane_plane(&p.frame, &q.frame, region);
        }
        (Surface::Plane(p), Surface::Cylinder(c)) => {
            return plane::plane_cylinder(&p.frame, c, region);
        }
        (Surface::Plane(p), Surface::Sphere(s)) => return plane::plane_sphere(&p.frame, s),
        (Surface::Plane(p), Surface::Torus(t)) => {
            if let Some(r) = sweep::plane_through_axis(t, p) {
                return Some(r);
            }
            let axis = axis_of(b)?;
            if parallel(p.frame.z, axis.dir) {
                return plane_across_axis(&p.frame, b, axis);
            }
            return sweep::intersect(a, b);
        }
        (Surface::Plane(p), Surface::Cone(c)) => {
            let axis = axis_of(b)?;
            if parallel(p.frame.z, axis.dir) {
                return plane_across_axis(&p.frame, b, axis);
            }
            return conic::plane_cone(p, c, region);
        }
        (Surface::Cylinder(x), Surface::Cylinder(y)) => {
            return cylinders::cylinder_cylinder(x, y, region).or_else(|| sweep::intersect(a, b));
        }
        _ => {}
    }
    let (xa, xb) = (axis_of(a)?, axis_of(b)?);
    let on_axis = |sp_origin: Point3, axis: Axis| {
        line_distance(sp_origin, axis.origin, axis.dir) <= tol::LINEAR
    };
    let symmetric = match (a, b) {
        (Surface::Sphere(sa), Surface::Sphere(sb)) => {
            let d = sb.frame.origin - sa.frame.origin;
            let dir = d.unit()?;
            coaxial(a, b, Axis { origin: sa.frame.origin, dir })
        }
        (Surface::Sphere(sp), _) => {
            on_axis(sp.frame.origin, xb).then(|| coaxial(a, b, xb)).flatten()
        }
        (_, Surface::Sphere(sp)) => {
            on_axis(sp.frame.origin, xa).then(|| coaxial(a, b, xa)).flatten()
        }
        _ => same_axis(xa, xb).then(|| coaxial(a, b, xa)).flatten(),
    };
    symmetric.or_else(|| sweep::intersect(a, b))
}
