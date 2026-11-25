// yack: Curve–surface and curve–curve intersection, signed distance to a surface, and ray hits.
// Adapted from springrep@a03e8a1 src/boolean/csi.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from boolean/csi.rs to intersect/csi.rs (M2.1, T2); paths only; then to yack-intersect (M3 W4), paths and visibility only; `.expect`/`.unwrap` as counted `panic!`s (gate 9).
// M3 hardening: split into submodules by kind (roots, pieces, line, spline, conic, curves), the
// dispatcher reading as a table of cases, checked access instead of indexing and no panics; the
// same arithmetic in the same order.
//! Curve–surface and curve–curve intersection, signed distance to a surface, and ray hits.
//!
//! Two crossings closer than the linear tolerance are one touching point: a curve whose excursion
//! past a surface is within [`tol::LINEAR`] touches it once, at the excursion's extreme.
//!
//! [`curve_surface`] picks a closed form by the pair of kinds: [`line`] holds a line against each
//! surface, [`spline`] a line against a spline surface, and [`conic`] circles and ellipses against
//! planes, cylinders and spheres. Everything else is sampled along the curve, by [`roots`]. The
//! curve–curve meetings are [`curves`]'s.

mod conic;
mod curves;
mod line;
mod pieces;
mod roots;
mod spline;
#[cfg(test)]
mod tests;

pub use curves::{GRAZE_GAP, curve_curve, curve_curve_near, grazing, on_three};

use self::{
    conic::{circle_parallel_cylinder, circle_plane, circle_sphere, ellipse_plane},
    line::{
        line_cone, line_cylinder, line_extrusion, line_plane, line_revolution, line_ruled,
        line_sphere, line_torus,
    },
    roots::{UNDERFLOW, evenly, sampled_roots, spans, with_ends},
    spline::line_spline,
};
use crate::{
    geom::{Curve, Line, Surface},
    hulls,
    math::{Aabb, Point3, Vec2, Vec3},
    tol,
};

/// A cone whose half angle's cosine is no more than this is a plane, with no quadric form.
const FLAT_CONE: f64 = 1e-9; // tol: a cosine that vanishes

/// The samples per span of a curve against a surface with no closed form: fewer where the curve's
/// breakpoints already split it.
const SAMPLES_PER_SPAN: usize = 12;
const SAMPLES_UNSPLIT: usize = 96;

/// The signed distance from `p` to `s`, positive on the side of the natural normal: exact near the
/// analytic surfaces, and along the projection's normal for the others.
pub fn signed_distance(s: &Surface, p: Point3) -> f64 {
    match s {
        Surface::Plane(pl) => (p - pl.frame.origin).dot(pl.frame.z),
        Surface::Cylinder(c) => {
            let l = c.frame.to_local(p);
            l.x.hypot(l.y) - c.radius
        }
        Surface::Sphere(sp) => (p - sp.frame.origin).norm() - sp.radius,
        Surface::Cone(c) => {
            let l = c.frame.to_local(p);
            let (sa, ca) = c.half_angle.sin_cos();
            (l.x.hypot(l.y) - c.radius) * ca - l.z * sa
        }
        Surface::Torus(t) => {
            let l = t.frame.to_local(p);
            (l.x.hypot(l.y) - t.major).hypot(l.z) - t.minor
        }
        _ => {
            let uv = s.project(p);
            (p - s.eval(uv.x, uv.y)).dot(s.normal(uv.x, uv.y))
        }
    }
}

/// The signed distance to `s` along a path of points: each projection onto a B-spline surface
/// continues from the last point's where the last point lay near enough the surface, and the
/// step from it is short enough, for that to find the nearest point.
pub struct Tracker<'a> {
    s: &'a Surface,
    /// The last point, its parameters on the surface and its signed distance.
    last: Option<(Point3, Vec2, f64)>,
}

impl<'a> Tracker<'a> {
    pub fn new(s: &'a Surface) -> Tracker<'a> {
        Tracker { s, last: None }
    }

    /// The signed distance from `p` to the surface.
    pub fn at(&mut self, p: Point3) -> f64 {
        let Surface::Nurbs(n) = self.s else {
            return signed_distance(self.s, p);
        };
        let uv = match self.last {
            Some((q, uv, d)) if d.abs() + q.dist(p) <= n.near_radius() => n.project_near(p, uv),
            _ => n.project(p),
        };
        let d = (p - n.eval(uv.x, uv.y)).dot(self.s.normal(uv.x, uv.y));
        self.last = Some((p, uv, d));
        d
    }
}

/// Where a curve meets a surface: the parameters of its crossing and touching points in its range,
/// or `on_surface` when the whole curve lies in the surface.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Hits {
    pub ts: Vec<f64>,
    pub on_surface: bool,
}

/// `t` held to `range`, where it lies within the parameter tolerance of it at the curve's speed.
fn in_range(c: &Curve, t: f64, range: (f64, f64)) -> Option<f64> {
    let pt = tol::param(c.d1(t).1.norm());
    (t >= range.0 - pt && t <= range.1 + pt).then(|| t.clamp(range.0, range.1))
}

/// Where the curve, over `range`, meets the surface.
pub fn curve_surface(c: &Curve, range: (f64, f64), s: &Surface) -> Hits {
    let raw = match (c, s) {
        (Curve::Line(l), Surface::Plane(p)) => line_plane(l.origin, l.dir, p),
        (Curve::Line(l), Surface::Cylinder(cyl)) => line_cylinder(l.origin, l.dir, cyl),
        (Curve::Line(l), Surface::Torus(t)) => line_torus(l.origin, l.dir, t, range),
        (Curve::Circle(ci), Surface::Sphere(sp)) => return circle_sphere(ci, sp, range),
        // A line along an extrusion's generators keeps one distance from it.
        (Curve::Line(l), Surface::Extrusion(e)) if l.dir.cross(e.dir).norm() <= tol::ANGULAR => {
            Hits { ts: Vec::new(), on_surface: signed_distance(s, l.origin).abs() <= tol::LINEAR }
        }
        (Curve::Line(l), Surface::Extrusion(e)) => line_extrusion(l.origin, l.dir, e),
        (Curve::Line(l), Surface::Revolution(r)) => match line_revolution(l.origin, l.dir, r) {
            Some(hits) => hits,
            None => return numeric_hits(c, range, s),
        },
        (Curve::Line(l), Surface::Ruled(r)) => match line_ruled(l.origin, l.dir, r) {
            Some(hits) => hits,
            None => return numeric_hits(c, range, s),
        },
        (Curve::Line(l), Surface::Nurbs(ns)) => match line_spline(l.origin, l.dir, ns, range) {
            Some(hits) => hits,
            None => return numeric_hits(c, range, s),
        },
        (Curve::Line(l), Surface::Sphere(sp)) => line_sphere(l.origin, l.dir, s, sp),
        (Curve::Line(l), Surface::Cone(cone)) if cone.half_angle.cos().abs() > FLAT_CONE => {
            line_cone(l.origin, l.dir, s, cone)
        }
        (Curve::Circle(ci), Surface::Plane(p)) => return circle_plane(ci, p, range),
        (Curve::Ellipse(e), Surface::Plane(p)) => return ellipse_plane(e, p, range),
        (Curve::Circle(ci), Surface::Cylinder(cyl))
            if ci.frame.z.cross(cyl.frame.z).norm() <= tol::ANGULAR =>
        {
            return circle_parallel_cylinder(ci, cyl, range);
        }
        _ => return numeric_hits(c, range, s),
    };
    if raw.on_surface {
        return raw;
    }
    let ts = raw.ts.into_iter().filter_map(|t| in_range(c, t, range)).collect();
    Hits { ts, on_surface: false }
}

/// Where the curve meets a surface with no closed form against it, from samples along the curve.
fn numeric_hits(c: &Curve, range: (f64, f64), s: &Surface) -> Hits {
    let cuts = with_ends(c.breakpoints(range.0, range.1), range);
    let n = if cuts.len() > 2 { SAMPLES_PER_SPAN } else { SAMPLES_UNSPLIT };
    let mut track = Tracker::new(s);
    let mut f = |t: f64| track.at(c.eval(t));
    let Surface::Nurbs(ns) = s else {
        let (ts, on) = sampled_roots(&mut f, &cuts, n);
        return Hits { ts, on_surface: on };
    };
    // The curve can meet the surface only where a stretch of it between samples comes near a
    // patch's box: only runs of such stretches are sampled, at the same points.
    let samples: Vec<f64> =
        spans(&cuts).flat_map(|(a, b)| evenly(a, b, n)).chain(std::iter::once(range.1)).collect();
    let mut hull = Aabb::EMPTY;
    for q in ns.patches() {
        hull.union(&q.bbox);
    }
    let meets = |a: &Aabb, b: &Aabb| {
        let (a_min, a_max, b_min, b_max) =
            (a.min.to_array(), a.max.to_array(), b.min.to_array(), b.max.to_array());
        a_min.into_iter().zip(b_max).zip(b_min.into_iter().zip(a_max)).all(
            |((a_lo, b_hi), (b_lo, a_hi))| a_lo <= b_hi + tol::LINEAR && b_lo <= a_hi + tol::LINEAR,
        )
    };
    let near = |t0: f64, t1: f64| {
        let bb = c.bbox(t0, t1);
        meets(&bb, &hull) && ns.patches().iter().any(|q| meets(&bb, &q.bbox))
    };
    let (mut ts, mut on) = (Vec::new(), true);
    let mut run: Vec<f64> = Vec::new();
    for stretch in samples.windows(2) {
        let &[t0, t1] = stretch else { continue };
        if near(t0, t1) {
            if run.is_empty() {
                run.push(t0);
            }
            run.push(t1);
            continue;
        }
        on = false;
        if !run.is_empty() {
            ts.extend(sampled_roots(&mut f, &run, 1).0);
            run.clear();
        }
    }
    if !run.is_empty() {
        let (r, all) = sampled_roots(&mut f, &run, 1);
        ts.extend(r);
        on &= all;
    }
    if on {
        return Hits { ts: Vec::new(), on_surface: true };
    }
    Hits { ts, on_surface: false }
}

/// The distance from `p` to the curve over `range`, and the parameter of the nearest point.
pub fn curve_distance(c: &Curve, range: (f64, f64), p: Point3) -> (f64, f64) {
    let t = c.project(p, range.0, range.1);
    (c.eval(t).dist(p), t)
}

/// Whether `p` lies within `reach` of the curve over `range`, as [`curve_distance`] measures it:
/// a point farther than that from every piece's hull box is farther from the curve, unprojected.
pub fn within(c: &Curve, range: (f64, f64), p: Point3, reach: f64) -> bool {
    if let Some(pieces) = hulls::pieces(c, range) {
        let mut bb = Aabb::EMPTY;
        bb.add(p);
        if pieces.iter().all(|q| hulls::gap(&q.hull, &bb) > reach) {
            return false;
        }
    }
    curve_distance(c, range, p).0 <= reach
}

/// The real roots of `a·t² + b·t + c`, by the stable form; the one root of a linear `b·t + c`.
fn quadratic_roots(a: f64, b: f64, c: f64) -> Vec<f64> {
    let disc = b * b - 4.0 * a * c;
    if a.abs() < UNDERFLOW || disc < 0.0 {
        return if a.abs() < UNDERFLOW && b.abs() > UNDERFLOW { vec![-c / b] } else { Vec::new() };
    }
    let q = -0.5 * (b + b.signum() * disc.sqrt());
    let mut roots = vec![q / a];
    if q.abs() > UNDERFLOW {
        roots.push(c / q);
    }
    roots
}

/// The parameters `t > 0` where the ray `p + t·d` (`d` unit) crosses the surface, up to `t_max`.
pub fn ray_surface(s: &Surface, p: Point3, d: Vec3, t_max: f64) -> Vec<f64> {
    match s {
        Surface::Plane(pl) => {
            let dn = d.dot(pl.frame.z);
            if dn.abs() < UNDERFLOW {
                Vec::new()
            } else {
                vec![(pl.frame.origin - p).dot(pl.frame.z) / dn]
            }
        }
        Surface::Cylinder(c) => {
            let lo = c.frame.to_local(p);
            let ld = c.frame.dir_to_local(d);
            quadratic_roots(
                ld.x * ld.x + ld.y * ld.y,
                2.0 * (lo.x * ld.x + lo.y * ld.y),
                lo.x * lo.x + lo.y * lo.y - c.radius * c.radius,
            )
        }
        Surface::Sphere(sp) => {
            let oc = p - sp.frame.origin;
            quadratic_roots(1.0, 2.0 * oc.dot(d), oc.norm2() - sp.radius * sp.radius)
        }
        Surface::Cone(cone) if cone.half_angle.cos().abs() > FLAT_CONE => {
            let lo = cone.frame.to_local(p);
            let ld = cone.frame.dir_to_local(d);
            let k = cone.half_angle.tan();
            let r0 = cone.radius + k * lo.z;
            quadratic_roots(
                ld.x * ld.x + ld.y * ld.y - k * k * ld.z * ld.z,
                2.0 * (lo.x * ld.x + lo.y * ld.y - k * r0 * ld.z),
                lo.x * lo.x + lo.y * lo.y - r0 * r0,
            )
        }
        // Every other surface by the routine the booleans use: exact where a closed form or a
        // search that cannot step over a crossing exists, sampled only where none does.
        _ => {
            let line = Curve::Line(Line { origin: p, dir: d });
            curve_surface(&line, (0.0, t_max), s).ts
        }
    }
}
