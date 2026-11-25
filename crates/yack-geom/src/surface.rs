// yack: Surfaces: the analytic five in closed form, and the swept and ruled surfaces built on curves.
// Adapted from springrep@a03e8a1 src/geom/surface.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/geom/surface.rs to yack-geom (M3 W3). M3 hardening: named
// constants and bindings, iterators instead of indexing, and an extrusion's projection asks for
// its spline once instead of a counted `panic!`; the same arithmetic in the same order.
//! Surfaces: the analytic five in closed form, and the swept and ruled surfaces built on curves.
//!
//! Each surface's natural normal is `∂S/∂u × ∂S/∂v`; a face whose material lies on that side is
//! not reversed. On every surface swept about an axis `u` is the angle about it.

use std::f64::consts::TAU;
use std::iter::once;

use super::{Curve, NurbsSurface};
use crate::{
    math::{Frame, Point3, Transform, Vec2, Vec3},
    numeric::solve2,
    tol,
};

#[derive(Clone, Debug, PartialEq)]
pub struct Plane {
    pub frame: Frame,
}

/// `origin + radius·radial(u) + v·z`.
#[derive(Clone, Debug, PartialEq)]
pub struct Cylinder {
    pub frame: Frame,
    pub radius: f64,
}

/// `origin + (radius + v·sin a)·radial(u) + v·cos a·z`, `a` the signed half angle, so `v` is arc
/// length along a generator and `radius` the radius at `v = 0`.
#[derive(Clone, Debug, PartialEq)]
pub struct Cone {
    pub frame: Frame,
    pub radius: f64,
    pub half_angle: f64,
}

/// `origin + radius·(cos v·radial(u) + sin v·z)`, `v` the latitude.
#[derive(Clone, Debug, PartialEq)]
pub struct Sphere {
    pub frame: Frame,
    pub radius: f64,
}

/// `origin + (major + minor·cos v)·radial(u) + minor·sin v·z`.
#[derive(Clone, Debug, PartialEq)]
pub struct Torus {
    pub frame: Frame,
    pub major: f64,
    pub minor: f64,
}

/// `curve(u) + v·dir`, `dir` a unit vector.
#[derive(Clone, Debug, PartialEq)]
pub struct Extrusion {
    pub curve: Curve,
    pub dir: Vec3,
}

/// `curve(v)` turned by `u` about the line through `origin` along the unit vector `axis`.
#[derive(Clone, Debug, PartialEq)]
pub struct Revolution {
    pub curve: Curve,
    pub origin: Point3,
    pub axis: Vec3,
}

/// `(1 − v)·a(u) + v·b(u)`, each curve's range mapped onto `u ∈ [0, 1]`.
#[derive(Clone, Debug, PartialEq)]
pub struct Ruled {
    pub a: Curve,
    pub a_range: (f64, f64),
    pub b: Curve,
    pub b_range: (f64, f64),
}

#[derive(Clone, Debug, PartialEq)]
pub enum Surface {
    Plane(Plane),
    Cylinder(Cylinder),
    Cone(Cone),
    Sphere(Sphere),
    Torus(Torus),
    Extrusion(Box<Extrusion>),
    Revolution(Box<Revolution>),
    Ruled(Box<Ruled>),
    /// A rational B-spline surface of any degree, as STEP files carry them.
    Nurbs(Box<NurbsSurface>),
}

/// A point this near a frame's axis, in both local coordinates, has angle zero about it.
const ON_AXIS: f64 = 1e-300; // tol: guards `atan2` at the origin, not a geometric tolerance

/// A derivatives' cross product shorter than this is degenerate: no normal is read from it.
const DEGENERATE_CROSS: f64 = 1e-12; // tol: squared-length scale of a vanishing normal

/// How far a degenerate normal's probes step off the point, in parameter.
const POLE_PROBE: f64 = 1e-6; // tol: parameter offset to read a limit normal

/// Two points within this fraction of [`tol::LINEAR`] are one point: the hint's `u` names a pole.
const POLE_FRACTION: f64 = 1e-3; // tol: fraction of the linear tolerance

/// The most Newton steps a projection takes.
const PROJECTION_STEPS: usize = 60;

/// The most Newton steps along an extrusion's curve.
const EXTRUSION_STEPS: usize = 30;

/// A Newton step this small, relative to the parameters, has converged.
const STEP_DONE: f64 = 1e-15; // tol: relative parameter rounding

/// A Newton step that moves the point farther than this relative rounding is refused.
const FARTHER: f64 = 1e-12; // tol: relative distance rounding

/// The absolute slack on that comparison, for a point already on the surface.
const DISTANCE_ROUNDING: f64 = 1e-15; // tol: absolute distance rounding

/// An extrusion's Newton iteration whose last step is below this, relative to `u`, has settled.
const EXTRUSION_SETTLED: f64 = 1e-9; // tol: relative parameter rounding

/// The largest parameter magnitude searched on a curve without a finite domain (a line).
const SEARCH_EXTENT: f64 = 1e4;

/// The seeds each smooth piece of a curve gives a projection's search.
const SEEDS_PER_PIECE: u32 = 8;

/// The derivative of [`Frame::radial`] by `u`.
fn d_radial(frame: &Frame, u: f64) -> Vec3 {
    frame.y * u.cos() - frame.x * u.sin()
}

/// The angle of `local` about the frame's `z`, in [0, 2π); zero on the axis.
fn angle_of(local: Vec3) -> f64 {
    if local.x.abs() < ON_AXIS && local.y.abs() < ON_AXIS {
        0.0
    } else {
        local.y.atan2(local.x).rem_euclid(TAU)
    }
}

impl Surface {
    pub fn kind(&self) -> &'static str {
        match self {
            Surface::Plane(_) => "plane",
            Surface::Cylinder(_) => "cylinder",
            Surface::Cone(_) => "cone",
            Surface::Sphere(_) => "sphere",
            Surface::Torus(_) => "torus",
            Surface::Extrusion(_) => "extrusion",
            Surface::Revolution(_) => "revolution",
            Surface::Ruled(_) => "ruled",
            Surface::Nurbs(_) => "nurbs",
        }
    }

    pub fn eval(&self, u: f64, v: f64) -> Point3 {
        self.d1(u, v).0
    }

    /// The point and its two first partial derivatives.
    pub fn d1(&self, u: f64, v: f64) -> (Point3, Vec3, Vec3) {
        match self {
            Surface::Plane(plane) => (plane.frame.at(u, v), plane.frame.x, plane.frame.y),
            Surface::Cylinder(cylinder) => {
                let f = &cylinder.frame;
                let radial = f.radial(u);
                (
                    f.origin + radial * cylinder.radius + f.z * v,
                    d_radial(f, u) * cylinder.radius,
                    f.z,
                )
            }
            Surface::Cone(cone) => {
                let f = &cone.frame;
                let (sin, cos) = cone.half_angle.sin_cos();
                let rho = cone.radius + v * sin;
                let radial = f.radial(u);
                (
                    f.origin + radial * rho + f.z * (v * cos),
                    d_radial(f, u) * rho,
                    radial * sin + f.z * cos,
                )
            }
            Surface::Sphere(sphere) => {
                let f = &sphere.frame;
                let (sin, cos) = v.sin_cos();
                let radial = f.radial(u);
                let r = sphere.radius;
                (
                    f.origin + (radial * cos + f.z * sin) * r,
                    d_radial(f, u) * (r * cos),
                    (f.z * cos - radial * sin) * r,
                )
            }
            Surface::Torus(torus) => {
                let f = &torus.frame;
                let (sin, cos) = v.sin_cos();
                let radial = f.radial(u);
                let rho = torus.major + torus.minor * cos;
                (
                    f.origin + radial * rho + f.z * (torus.minor * sin),
                    d_radial(f, u) * rho,
                    (f.z * cos - radial * sin) * torus.minor,
                )
            }
            Surface::Extrusion(extrusion) => {
                let (point, tangent) = extrusion.curve.d1(u);
                (point + extrusion.dir * v, tangent, extrusion.dir)
            }
            Surface::Revolution(revolution) => {
                let (q, dq) = revolution.curve.d1(v);
                let rotation = Transform::rotation(revolution.origin, revolution.axis, u);
                let point = rotation.point(q);
                (point, revolution.axis.cross(point - revolution.origin), rotation.vector(dq))
            }
            Surface::Ruled(ruled) => {
                let a_span = ruled.a_range.1 - ruled.a_range.0;
                let b_span = ruled.b_range.1 - ruled.b_range.0;
                let (pa, da) = ruled.a.d1(ruled.a_range.0 + u * a_span);
                let (pb, db) = ruled.b.d1(ruled.b_range.0 + u * b_span);
                (pa.lerp(pb, v), da * (a_span * (1.0 - v)) + db * (b_span * v), pb - pa)
            }
            Surface::Nurbs(nurbs) => nurbs.d1(u, v),
        }
    }

    /// The unit natural normal. At a pole, where the derivatives' cross product vanishes, it is
    /// the limit from inside the parameter domain.
    pub fn normal(&self, u: f64, v: f64) -> Vec3 {
        match self {
            Surface::Plane(plane) => plane.frame.z,
            Surface::Cylinder(cylinder) => cylinder.frame.radial(u),
            Surface::Cone(cone) => {
                let (sin, cos) = cone.half_angle.sin_cos();
                let rho = cone.radius + v * sin;
                let n = cone.frame.radial(u) * cos - cone.frame.z * sin;
                if rho < 0.0 { -n } else { n }
            }
            Surface::Sphere(sphere) => {
                let (sin, cos) = v.sin_cos();
                sphere.frame.radial(u) * cos + sphere.frame.z * sin
            }
            Surface::Torus(torus) => {
                let (sin, cos) = v.sin_cos();
                let n = torus.frame.radial(u) * cos + torus.frame.z * sin;
                if torus.major + torus.minor * cos < 0.0 { -n } else { n }
            }
            _ => {
                let (_, su, sv) = self.d1(u, v);
                if let Some(n) = natural_normal(su, sv) {
                    return n;
                }
                // At a pole where `su` vanishes it grows as `suv·(v − v_pole)`, so the normal is
                // the limit of `suv × sv` taken from inside the domain (and alike for `sv`).
                if let Surface::Nurbs(nurbs) = self {
                    let ((u0, u1), (v0, v1)) = nurbs.domain();
                    let [_, su, sv, _, suv, _] = nurbs.d2(u, v);
                    let (limit, past_middle) = if su.norm() <= sv.norm() {
                        (suv.cross(sv), v - v0 > v1 - v)
                    } else {
                        (su.cross(suv), u - u0 > u1 - u)
                    };
                    if let Some(limit) = limit.unit() {
                        return if past_middle { -limit } else { limit };
                    }
                }
                let probes =
                    [(0.0, POLE_PROBE), (0.0, -POLE_PROBE), (POLE_PROBE, 0.0), (-POLE_PROBE, 0.0)];
                probes
                    .into_iter()
                    .find_map(|(du, dv)| {
                        let (_, su, sv) = self.d1(u + du, v + dv);
                        su.cross(sv).unit()
                    })
                    .unwrap_or(Vec3::ZERO)
            }
        }
    }

    pub fn u_period(&self) -> Option<f64> {
        match self {
            Surface::Cylinder(_)
            | Surface::Cone(_)
            | Surface::Sphere(_)
            | Surface::Torus(_)
            | Surface::Revolution(_) => Some(TAU),
            Surface::Extrusion(extrusion) => extrusion.curve.period(),
            Surface::Nurbs(nurbs) => nurbs.u_period(),
            Surface::Plane(_) | Surface::Ruled(_) => None,
        }
    }

    pub fn v_period(&self) -> Option<f64> {
        match self {
            Surface::Torus(_) => Some(TAU),
            Surface::Revolution(revolution) => revolution.curve.period(),
            Surface::Nurbs(nurbs) => nurbs.v_period(),
            _ => None,
        }
    }

    /// Parameters strictly inside (u0, u1) where the surface may lose smoothness along `u`.
    pub fn u_breakpoints(&self, u0: f64, u1: f64) -> Vec<f64> {
        match self {
            Surface::Extrusion(extrusion) => extrusion.curve.breakpoints(u0, u1),
            Surface::Ruled(ruled) => {
                // A side's breakpoints, mapped from its own range onto [0, 1].
                let side = |range: (f64, f64), curve: &Curve| -> Vec<f64> {
                    let span = range.1 - range.0;
                    let (lo, hi) = (range.0 + u0 * span, range.0 + u1 * span);
                    let (lo, hi) = (lo.min(hi), lo.max(hi));
                    curve.breakpoints(lo, hi).into_iter().map(|t| (t - range.0) / span).collect()
                };
                let mut out = side(ruled.a_range, &ruled.a);
                out.extend(side(ruled.b_range, &ruled.b));
                out.sort_by(f64::total_cmp);
                out.dedup();
                out
            }
            Surface::Nurbs(nurbs) => nurbs.u_breakpoints(u0, u1),
            _ => Vec::new(),
        }
    }

    /// Parameters strictly inside (v0, v1) where the surface may lose smoothness along `v`.
    pub fn v_breakpoints(&self, v0: f64, v1: f64) -> Vec<f64> {
        match self {
            Surface::Revolution(revolution) => revolution.curve.breakpoints(v0, v1),
            Surface::Nurbs(nurbs) => nurbs.v_breakpoints(v0, v1),
            _ => Vec::new(),
        }
    }

    pub fn transformed(&self, t: &Transform) -> Surface {
        match self {
            Surface::Plane(plane) => Surface::Plane(Plane { frame: plane.frame.transformed(t) }),
            Surface::Cylinder(cylinder) => {
                Surface::Cylinder(Cylinder { frame: cylinder.frame.transformed(t), ..*cylinder })
            }
            Surface::Cone(cone) => {
                Surface::Cone(Cone { frame: cone.frame.transformed(t), ..*cone })
            }
            Surface::Sphere(sphere) => {
                Surface::Sphere(Sphere { frame: sphere.frame.transformed(t), ..*sphere })
            }
            Surface::Torus(torus) => {
                Surface::Torus(Torus { frame: torus.frame.transformed(t), ..*torus })
            }
            Surface::Extrusion(extrusion) => Surface::Extrusion(Box::new(Extrusion {
                curve: extrusion.curve.transformed(t),
                dir: t.vector(extrusion.dir),
            })),
            Surface::Revolution(revolution) => Surface::Revolution(Box::new(Revolution {
                curve: revolution.curve.transformed(t),
                origin: t.point(revolution.origin),
                axis: t.vector(revolution.axis),
            })),
            Surface::Ruled(ruled) => Surface::Ruled(Box::new(Ruled {
                a: ruled.a.transformed(t),
                a_range: ruled.a_range,
                b: ruled.b.transformed(t),
                b_range: ruled.b_range,
            })),
            Surface::Nurbs(nurbs) => Surface::Nurbs(Box::new(nurbs.transformed(t))),
        }
    }

    /// The parameters of the surface point nearest `p`. A periodic parameter comes back in its
    /// first period.
    pub fn project(&self, p: Point3) -> Vec2 {
        match self {
            Surface::Plane(plane) => {
                let local = plane.frame.to_local(p);
                Vec2::new(local.x, local.y)
            }
            Surface::Cylinder(cylinder) => {
                let local = cylinder.frame.to_local(p);
                Vec2::new(angle_of(local), local.z)
            }
            Surface::Cone(cone) => {
                let local = cone.frame.to_local(p);
                let rho = local.x.hypot(local.y);
                let (sin, cos) = cone.half_angle.sin_cos();
                Vec2::new(angle_of(local), (rho - cone.radius) * sin + local.z * cos)
            }
            Surface::Sphere(sphere) => {
                let local = sphere.frame.to_local(p);
                Vec2::new(angle_of(local), local.z.atan2(local.x.hypot(local.y)))
            }
            Surface::Torus(torus) => {
                let local = torus.frame.to_local(p);
                let rho = local.x.hypot(local.y);
                Vec2::new(angle_of(local), local.z.atan2(rho - torus.major).rem_euclid(TAU))
            }
            Surface::Extrusion(extrusion) => {
                let curve = &extrusion.curve;
                match curve.spline() {
                    Some(spline) => {
                        let (a, b) = curve.domain();
                        // The nearest point lies on the generator through the curve point nearest
                        // `p` seen along the extrusion.
                        let u = spline.nearest(p, Some(extrusion.dir), a, b);
                        let v = (p - curve.eval(u)).dot(extrusion.dir);
                        self.refine_projection(p, once(Vec2::new(u, v)))
                    }
                    None => {
                        let (a, b) = finite_domain(curve);
                        let seeds = curve_seeds(curve, a, b)
                            .into_iter()
                            .map(|u| Vec2::new(u, (p - curve.eval(u)).dot(extrusion.dir)));
                        self.refine_projection(p, seeds)
                    }
                }
            }
            Surface::Revolution(revolution) => {
                let curve = &revolution.curve;
                let (a, b) = finite_domain(curve);
                let frame =
                    Frame::from_axis(revolution.origin, revolution.axis).unwrap_or(Frame::WORLD);
                let angle = angle_of(frame.to_local(p));
                let seeds = curve_seeds(curve, a, b).into_iter().map(|v| {
                    let q = frame.to_local(curve.eval(v));
                    Vec2::new((angle - angle_of(q)).rem_euclid(TAU), v)
                });
                self.refine_projection(p, seeds)
            }
            Surface::Ruled(_) => {
                let seeds = (0..=16)
                    .flat_map(|i| (0..=4).map(move |j| Vec2::new(i as f64 / 16.0, j as f64 / 4.0)));
                self.refine_projection(p, seeds)
            }
            Surface::Nurbs(nurbs) => nurbs.project(p),
        }
    }

    /// [`Surface::project`] for a point that should lie on the surface: a spline searches from
    /// more seeds where the nearest few settle off it ([`NurbsSurface::project_onto`]).
    pub fn project_onto(&self, p: Point3) -> Vec2 {
        match self {
            Surface::Nurbs(nurbs) => nurbs.project_onto(p),
            _ => self.project(p),
        }
    }

    /// As [`Surface::project`], with each periodic parameter moved by whole periods to lie
    /// nearest `hint`. At a pole, where every `u` is the same point, `u` stays at the hint's, so a
    /// curve through the pole keeps the `u` it arrived with.
    pub fn project_near(&self, p: Point3, hint: Vec2) -> Vec2 {
        self.project_near_at(p, hint).0
    }

    /// [`Surface::project_near`] with the surface point there and its unit natural normal, from
    /// the one evaluation that places the parameters.
    pub fn project_near_at(&self, p: Point3, hint: Vec2) -> (Vec2, Point3, Vec3) {
        let uv = self.project_near_raw(p, hint);
        let (point, su, sv) = self.d1(uv.x, uv.y);
        let out = self.toward(uv, hint, su);
        let normal = match self {
            Surface::Plane(_)
            | Surface::Cylinder(_)
            | Surface::Cone(_)
            | Surface::Sphere(_)
            | Surface::Torus(_) => self.normal(out.x, out.y),
            _ => natural_normal(su, sv).unwrap_or_else(|| self.normal(out.x, out.y)),
        };
        (out, point, normal)
    }

    /// [`Surface::project_near`] with the partial derivatives there, a spline's from its span
    /// polynomials.
    pub fn project_near_d1(&self, p: Point3, hint: Vec2) -> (Vec2, Vec3, Vec3) {
        let uv = self.project_near_raw(p, hint);
        let (_, su, sv) = match self {
            Surface::Nurbs(nurbs) => nurbs.d1_by_polys(uv.x, uv.y),
            _ => self.d1(uv.x, uv.y),
        };
        (self.toward(uv, hint, su), su, sv)
    }

    /// The parameters [`Surface::project_near`] places `p` at, before they are moved toward `hint`.
    fn project_near_raw(&self, p: Point3, hint: Vec2) -> Vec2 {
        let on_surface = |uv: Vec2| self.eval(uv.x, uv.y).dist(p) <= tol::LINEAR;
        match self {
            // A point on the surface near the hint is found by Newton from the hint alone; the
            // global seed search is needed only when that lands off the surface.
            Surface::Extrusion(extrusion) if extrusion.curve.spline().is_some() => {
                match extrusion_near(extrusion, p, hint.x) {
                    Some(uv) if on_surface(uv) => uv,
                    _ => self.project(p),
                }
            }
            Surface::Extrusion(_) | Surface::Revolution(_) | Surface::Ruled(_) => {
                let local = self.refine_projection(p, once(hint));
                if on_surface(local) { local } else { self.project(p) }
            }
            Surface::Nurbs(nurbs) => nurbs.project_near(p, hint),
            _ => self.project(p),
        }
    }

    /// `uv` with each periodic parameter moved by whole periods to lie nearest `hint`, and `u` the
    /// hint's where the point is a pole: where the hint's `u` names the same point.
    fn toward(&self, uv: Vec2, hint: Vec2, su: Vec3) -> Vec2 {
        let shift = |x: f64, h: f64, period: Option<f64>| match period {
            Some(period) => x + ((h - x) / period).round() * period,
            None => x,
        };
        // A small `su` alone is not a pole: just beside one, the hint's `u` can be a whole
        // tolerance away, which a fit to half the tolerance cannot absorb.
        let pole =
            || self.eval(hint.x, uv.y).dist(self.eval(uv.x, uv.y)) <= POLE_FRACTION * tol::LINEAR;
        let u = if su.norm() <= tol::LINEAR && pole() {
            hint.x
        } else {
            shift(uv.x, hint.x, self.u_period())
        };
        Vec2::new(u, shift(uv.y, hint.y, self.v_period()))
    }

    /// Newton's method on the distance to `p` from the nearest of `seeds`, each step taken only
    /// if it comes no farther; periodic parameters are wrapped into their first period after.
    fn refine_projection(&self, p: Point3, seeds: impl Iterator<Item = Vec2>) -> Vec2 {
        let mut best = (f64::INFINITY, Vec2::ZERO);
        for seed in seeds {
            let distance = self.eval(seed.x, seed.y).dist(p);
            if distance < best.0 {
                best = (distance, seed);
            }
        }
        let mut uv = best.1;
        for _ in 0..PROJECTION_STEPS {
            let (q, su, sv) = self.d1(uv.x, uv.y);
            let r = q - p;
            let Some((du, dv)) =
                solve2(su.dot(su), su.dot(sv), su.dot(sv), sv.dot(sv), -su.dot(r), -sv.dot(r))
            else {
                break;
            };
            let next = Vec2::new(uv.x + du, uv.y + dv);
            let done = du.abs() + dv.abs() <= STEP_DONE * (1.0 + uv.x.abs() + uv.y.abs());
            if self.eval(next.x, next.y).dist(p) > r.norm() * (1.0 + FARTHER) + DISTANCE_ROUNDING {
                break;
            }
            uv = next;
            if done {
                break;
            }
        }
        let wrap = |x: f64, period: Option<f64>, start: f64| match period {
            Some(period) => start + (x - start).rem_euclid(period),
            None => x,
        };
        let u0 = match self {
            Surface::Extrusion(extrusion) => extrusion.curve.domain().0,
            _ => 0.0,
        };
        let v0 = match self {
            Surface::Revolution(revolution) => revolution.curve.domain().0,
            _ => 0.0,
        };
        Vec2::new(wrap(uv.x, self.u_period(), u0), wrap(uv.y, self.v_period(), v0))
    }
}

/// The unit normal `su × sv`, unless the cross product is too short to give one.
fn natural_normal(su: Vec3, sv: Vec3) -> Option<Vec3> {
    let cross = su.cross(sv);
    cross.unit().filter(|_| cross.norm() > DEGENERATE_CROSS)
}

/// The nearest point of an extrusion by Newton's method on the curve parameter alone, from `u`:
/// across the extrusion's direction the surface is its curve, and `v` follows in closed form.
fn extrusion_near(extrusion: &Extrusion, p: Point3, mut u: f64) -> Option<Vec2> {
    let dir = extrusion.dir;
    let flat = |w: Vec3| w - dir * w.dot(dir);
    let mut step = f64::INFINITY;
    for _ in 0..EXTRUSION_STEPS {
        let (point, d1, d2) = extrusion.curve.d2(u);
        let (r, d1) = (flat(point - p), flat(d1));
        let slope = r.dot(d1);
        let curvature = d1.norm2() + r.dot(flat(d2));
        if curvature <= 0.0 {
            return None;
        }
        step = slope / curvature;
        u -= step;
        if step.abs() <= STEP_DONE * (1.0 + u.abs()) {
            break;
        }
    }
    // Rounding may keep the last steps from vanishing; the caller checks the point is on the
    // extrusion.
    (step.abs() <= EXTRUSION_SETTLED * (1.0 + u.abs()))
        .then(|| Vec2::new(u, (p - extrusion.curve.eval(u)).dot(dir)))
}

/// The curve's domain, cut to [`SEARCH_EXTENT`] either side of zero.
fn finite_domain(curve: &Curve) -> (f64, f64) {
    let (a, b) = curve.domain();
    (a.max(-SEARCH_EXTENT), b.min(SEARCH_EXTENT))
}

/// Projection seeds along the curve over [a, b]: [`SEEDS_PER_PIECE`] equal steps over each
/// smooth piece, and `b`.
fn curve_seeds(curve: &Curve, a: f64, b: f64) -> Vec<f64> {
    let ends: Vec<f64> = once(a).chain(curve.breakpoints(a, b)).chain(once(b)).collect();
    let starts = ends.iter().copied();
    starts
        .clone()
        .zip(starts.skip(1))
        .flat_map(|(start, end)| {
            (0..SEEDS_PER_PIECE)
                .map(move |i| start + (end - start) * f64::from(i) / f64::from(SEEDS_PER_PIECE))
        })
        .chain(once(b))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{BSpline, Circle};

    fn frame() -> Frame {
        Frame::new(Vec3::new(1.0, -2.0, 0.5), Vec3::new(0.2, 0.3, 1.0), Vec3::X).unwrap()
    }

    fn all() -> Vec<Surface> {
        let f = frame();
        let spline = BSpline::interpolate(&[
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(2.0, 1.0, 0.0),
            Vec3::new(4.0, 0.5, 0.0),
            Vec3::new(6.0, 2.0, 0.0),
        ])
        .unwrap();
        let profile = Curve::BSpline(
            BSpline::interpolate(&[
                Vec3::new(5.0, 0.0, 0.0),
                Vec3::new(6.0, 0.0, 2.0),
                Vec3::new(5.5, 0.0, 4.0),
                Vec3::new(7.0, 0.0, 6.0),
            ])
            .unwrap(),
        );
        let circle = Curve::Circle(Circle { frame: Frame::WORLD, radius: 3.0 });
        vec![
            Surface::Plane(Plane { frame: f }),
            Surface::Cylinder(Cylinder { frame: f, radius: 4.0 }),
            Surface::Cone(Cone { frame: f, radius: 4.0, half_angle: 0.4 }),
            Surface::Sphere(Sphere { frame: f, radius: 4.0 }),
            Surface::Torus(Torus { frame: f, major: 6.0, minor: 2.0 }),
            Surface::Extrusion(Box::new(Extrusion {
                curve: Curve::BSpline(spline),
                dir: Vec3::new(0.0, 0.6, 0.8),
            })),
            Surface::Revolution(Box::new(Revolution {
                curve: profile,
                origin: Vec3::ZERO,
                axis: Vec3::Z,
            })),
            Surface::Ruled(Box::new(Ruled {
                a: circle.clone(),
                a_range: (0.0, TAU),
                b: circle.transformed(&Transform::translation(Vec3::new(0.0, 0.0, 5.0))),
                b_range: (0.0, TAU),
            })),
        ]
    }

    #[test]
    fn derivatives_and_normals_match_differences() {
        for s in all() {
            let (u, v) = (0.7, 0.4);
            let h = 1e-6;
            let (_, su, sv) = s.d1(u, v);
            let fu = (s.eval(u + h, v) - s.eval(u - h, v)) / (2.0 * h);
            let fv = (s.eval(u, v + h) - s.eval(u, v - h)) / (2.0 * h);
            assert!(su.dist(fu) < 1e-7, "{} su", s.kind());
            assert!(sv.dist(fv) < 1e-7, "{} sv", s.kind());
            let n = s.normal(u, v);
            assert!(n.dist(su.cross(sv).unit().unwrap()) < 1e-12, "{} normal", s.kind());
        }
    }

    #[test]
    fn a_spline_normal_at_a_pole_is_its_limit() {
        // At the octant's pole every `u` is the point (0, 0, r), where the sphere's normal is +z.
        let s = crate::octant(2.0);
        for u in [0.0, 0.3, 0.5, 1.0] {
            let n = s.normal(u, 1.0);
            assert!(n.dist(Vec3::Z) < 1e-14, "u {u}: {n:?}");
        }
    }

    #[test]
    fn projection_inverts_evaluation() {
        for s in all() {
            for &(u, v) in &[(0.7, 0.4), (2.5, 0.9), (5.9, 0.1)] {
                let (u, v) = match s {
                    Surface::Ruled(_) => (u / 6.0, v),
                    Surface::Extrusion(_) | Surface::Revolution(_) => (u, v * 4.0),
                    _ => (u, v),
                };
                let p = s.eval(u, v);
                let uv = s.project_near(p, Vec2::new(u, v));
                assert!(s.eval(uv.x, uv.y).dist(p) < 1e-9, "{} at {u},{v}: {uv:?}", s.kind());
                assert!((uv.x - u).abs() < 1e-7 && (uv.y - v).abs() < 1e-7, "{}", s.kind());
            }
        }
    }
}
