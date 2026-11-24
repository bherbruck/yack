// yack: ISO 10303-42 curves and surfaces: exact evaluation, first derivatives and point inversion.
//! The geometry a STEP B-rep is built on, parametrised exactly as ISO 10303-42 defines each entity:
//! - `plane`: C + u·x + v·y
//! - `cylindrical_surface`: C + R(cos u·x + sin u·y) + v·z
//! - `conical_surface`: C + (R + v·tan α)(cos u·x + sin u·y) + v·z, so `v` is height along the axis
//! - `spherical_surface`: C + R cos v (cos u·x + sin u·y) + R sin v·z
//! - `toroidal_surface`: C + (R + r cos v)(cos u·x + sin u·y) + r sin v·z
//! - `surface_of_linear_extrusion`: c(u) + v·V
//! - `surface_of_revolution`: c(v) turned by u about the axis
//! - `b_spline_surface_with_knots` (rational or not)
//!
//! Curves: `line` (P + t·V), `circle`, `ellipse` (angle t), `b_spline_curve_with_knots`.

use std::f64::consts::{PI, TAU};

use yack_math::{Point3, Vec3};

use yack_base::Result;

use crate::bspline;
use crate::read::bad;

/// A right-handed frame: origin and unit axes.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Frame {
    pub(crate) o: Point3,
    pub(crate) x: Vec3,
    pub(crate) y: Vec3,
    pub(crate) z: Vec3,
}

impl Frame {
    fn local(&self, p: Point3) -> (f64, f64, f64) {
        let d = p - self.o;
        (d.dot(self.x), d.dot(self.y), d.dot(self.z))
    }
    fn radial(&self, u: f64) -> (Vec3, Vec3) {
        let (s, c) = u.sin_cos();
        (self.x * c + self.y * s, self.y * c - self.x * s)
    }
}

/// An edge curve.
#[derive(Clone, Debug, PartialEq)]
pub(crate) enum Curve {
    Line { p: Point3, v: Vec3 },
    Circle { f: Frame, r: f64 },
    Ellipse { f: Frame, a: f64, b: f64 },
    Spline(bspline::Curve),
}

impl Curve {
    /// The point and first derivative at `t`.
    pub(crate) fn d1(&self, t: f64) -> (Point3, Vec3) {
        match self {
            Curve::Line { p, v } => (*p + *v * t, *v),
            Curve::Circle { f, r } => {
                let (e, de) = f.radial(t);
                (f.o + e * *r, de * *r)
            }
            Curve::Ellipse { f, a, b } => {
                let (s, c) = t.sin_cos();
                (f.o + f.x * (a * c) + f.y * (b * s), f.y * (b * c) - f.x * (a * s))
            }
            Curve::Spline(c) => c.d1(t),
        }
    }

    /// The period of a closed curve's parameter (a circle's and an ellipse's 2π).
    pub(crate) fn period(&self) -> Option<f64> {
        match self {
            Curve::Circle { .. } | Curve::Ellipse { .. } => Some(TAU),
            Curve::Spline(c) => c.period,
            Curve::Line { .. } => None,
        }
    }

    /// The parameters strictly between `lo` and `hi` at which the curve's smoothness may break (a
    /// spline's knots) or past which one quadrature piece would be too long (an eighth of a turn).
    pub(crate) fn breaks(&self, lo: f64, hi: f64) -> Result<Vec<f64>> {
        match self {
            Curve::Line { .. } => Ok(Vec::new()),
            Curve::Circle { .. } | Curve::Ellipse { .. } => angular(lo, hi),
            Curve::Spline(c) => knots_between(&c.basis.knots, c.basis.domain(), c.period, lo, hi),
        }
    }

    /// The curve's own parameter range (unbounded for a line).
    pub(crate) fn domain(&self) -> (f64, f64) {
        match self {
            Curve::Line { .. } => (f64::NEG_INFINITY, f64::INFINITY),
            Curve::Circle { .. } | Curve::Ellipse { .. } => (0.0, TAU),
            Curve::Spline(c) => c.basis.domain(),
        }
    }

    /// The parameter of the curve's point nearest `q`, near `hint` where given.
    pub(crate) fn invert(&self, q: Point3, hint: Option<f64>) -> f64 {
        match self {
            Curve::Line { p, v } => {
                let vv = v.dot(*v);
                if vv > 0.0 { (q - *p).dot(*v) / vv } else { 0.0 }
            }
            Curve::Circle { f, .. } => near_period(angle(f.local(q).1, f.local(q).0), hint),
            Curve::Ellipse { f, a, b } => {
                let (lx, ly, _) = f.local(q);
                near_period(angle(ly / b, lx / a), hint)
            }
            Curve::Spline(c) => {
                let (a, b) = self.domain();
                let closed = c.period.is_some();
                let mut t = hint.unwrap_or_else(|| {
                    // Coarse search, then Newton.
                    (0..=64)
                        .map(|i| a + (b - a) * i as f64 / 64.0)
                        .min_by(|s, t| self.d1(*s).0.distance(q).total_cmp(&self.d1(*t).0.distance(q)))
                        .unwrap_or(a)
                });
                for _ in 0..50 {
                    let (p, d) = self.d1(t);
                    let dd = d.dot(d);
                    if dd <= 0.0 {
                        break;
                    }
                    let step = (q - p).dot(d) / dd;
                    t = if closed { t + step } else { (t + step).clamp(a, b) };
                    if step.abs() <= NEWTON_STEP * (b - a).abs().max(1.0) {
                        break;
                    }
                }
                match (c.period, hint) {
                    (Some(p), Some(h)) => t + ((h - t) / p).round() * p,
                    _ => t,
                }
            }
        }
    }
}

/// When a Newton step is this small, relative to the parameter range, it has converged.
const NEWTON_STEP: f64 = 1e-14; // tol: round-off in a parameter

/// How many times a Newton step that does not bring the point nearer is halved before the descent
/// stops.
const HALVINGS: usize = 6;

/// `atan2(y, x)` in `[0, 2π)` (0 at the origin).
fn angle(y: f64, x: f64) -> f64 {
    if x == 0.0 && y == 0.0 { 0.0 } else { y.atan2(x).rem_euclid(TAU) }
}

/// The longest quadrature piece along an angle: an eighth of a turn.
const ANGULAR_PIECE: f64 = PI / 4.0;

/// The most quadrature pieces one run may be cut into: a file whose parameters would need more
/// (a vast angle, a closed spline with a vanishing period) is refused, not integrated without end.
const MAX_PIECES: f64 = 10_000.0;

fn too_many(n: f64) -> yack_base::Error {
    bad(format!("a parameter run would need {n} quadrature pieces (more than {MAX_PIECES})"))
}

/// Equal steps strictly between `lo` and `hi`, none longer than [`ANGULAR_PIECE`].
fn angular(lo: f64, hi: f64) -> Result<Vec<f64>> {
    // Ascending whichever way the run goes, as every `breaks` is: a caller running down reverses
    // them (a run down a circle or a sphere's meridian once took its pieces in the wrong order, so
    // each spanned nearly the whole run and the quadrature lost digits).
    let (lo, hi) = if lo <= hi { (lo, hi) } else { (hi, lo) };
    let span = hi - lo;
    if !span.is_finite() {
        return Ok(Vec::new());
    }
    let n = (span / ANGULAR_PIECE).ceil().max(1.0);
    if n > MAX_PIECES {
        return Err(too_many(n));
    }
    let n = n as usize;
    Ok((1..n).map(|i| lo + (hi - lo) * i as f64 / n as f64).collect())
}

/// A spline's distinct knots strictly between `lo` and `hi` (with their copies a whole period on,
/// for a closed spline).
fn knots_between(knots: &[f64], domain: (f64, f64), period: Option<f64>, lo: f64, hi: f64) -> Result<Vec<f64>> {
    let (lo, hi) = if lo <= hi { (lo, hi) } else { (hi, lo) };
    let mut inner: Vec<f64> = knots.iter().copied().filter(|k| *k > domain.0 && *k < domain.1).collect();
    inner.dedup();
    let mut out = Vec::new();
    match period {
        Some(p) if p > 0.0 && lo.is_finite() && hi.is_finite() => {
            let (first, last) = (((lo - domain.0) / p).floor(), ((hi - domain.0) / p).ceil());
            // A run touches at most one period more than it spans whole: the periods looped over
            // below reach a period past each end, which the cap must not count (bench fillet/tee-
            // of-two-cylinders: a closed spline of 4094 spans run once round, counted as 12282).
            let per = (inner.len() + 1) as f64;
            let pieces = (((hi - lo) / p).ceil() + 1.0) * per;
            if !pieces.is_finite() || pieces > MAX_PIECES {
                return Err(too_many(pieces));
            }
            // The periods the loop runs over, bounded by themselves too: far from zero `first` and
            // `last` round apart from the span (at 1e20 by thousands of periods), and past i64 the
            // casts saturate.
            let looped = (last - first + 1.0) * per;
            let fits = |x: f64| x.abs() < i64::MAX as f64;
            if !(looped.is_finite() && looped <= MAX_PIECES + 2.0 * per && fits(first) && fits(last)) {
                return Err(too_many(looped));
            }
            for k in (first as i64)..=(last as i64) {
                let shift = k as f64 * p;
                out.extend(inner.iter().map(|x| x + shift).chain(std::iter::once(domain.0 + shift)).filter(|x| *x > lo && *x < hi));
            }
        }
        _ => out.extend(inner.into_iter().filter(|x| *x > lo && *x < hi)),
    }
    out.sort_by(f64::total_cmp);
    out.dedup();
    Ok(out)
}

/// `a` moved by whole turns to lie within half a turn of `hint`.
pub(crate) fn near_period(a: f64, hint: Option<f64>) -> f64 {
    match hint {
        Some(h) => a + ((h - a) / TAU).round() * TAU,
        None => a,
    }
}

/// A face's surface.
#[derive(Clone, Debug, PartialEq)]
pub(crate) enum Surface {
    Plane { f: Frame },
    Cylinder { f: Frame, r: f64 },
    Cone { f: Frame, r: f64, tan: f64 },
    Sphere { f: Frame, r: f64 },
    Torus { f: Frame, rmaj: f64, rmin: f64 },
    Extrusion { c: Curve, d: Vec3 },
    Revolution { c: Curve, o: Point3, axis: Vec3 },
    Spline(bspline::Surface),
}

impl Surface {
    /// The point and its two first partial derivatives at `(u, v)`.
    pub(crate) fn d1(&self, u: f64, v: f64) -> (Point3, Vec3, Vec3) {
        match self {
            Surface::Plane { f } => (f.o + f.x * u + f.y * v, f.x, f.y),
            Surface::Cylinder { f, r } => {
                let (e, de) = f.radial(u);
                (f.o + e * *r + f.z * v, de * *r, f.z)
            }
            Surface::Cone { f, r, tan } => {
                let (e, de) = f.radial(u);
                let rho = r + v * tan;
                (f.o + e * rho + f.z * v, de * rho, e * *tan + f.z)
            }
            Surface::Sphere { f, r } => {
                let (e, de) = f.radial(u);
                let (s, c) = v.sin_cos();
                (f.o + (e * c + f.z * s) * *r, de * (r * c), (f.z * c - e * s) * *r)
            }
            Surface::Torus { f, rmaj, rmin } => {
                let (e, de) = f.radial(u);
                let (s, c) = v.sin_cos();
                let rho = rmaj + rmin * c;
                (f.o + e * rho + f.z * (rmin * s), de * rho, (f.z * c - e * s) * *rmin)
            }
            Surface::Extrusion { c, d } => {
                let (p, dp) = c.d1(u);
                (p + *d * v, dp, *d)
            }
            Surface::Revolution { c, o, axis } => {
                let (p, dp) = c.d1(v);
                let rel = p - *o;
                let a = rel.dot(*axis);
                let rr = rel - *axis * a;
                let side = axis.cross(rr);
                let (s, co) = u.sin_cos();
                let pt = *o + *axis * a + rr * co + side * s;
                let su = side * co - rr * s;
                // d/dv: the same turn applied to the curve's tangent.
                let da = dp.dot(*axis);
                let drr = dp - *axis * da;
                let sv = *axis * da + drr * co + axis.cross(drr) * s;
                (pt, su, sv)
            }
            Surface::Spline(s) => s.d1(u, v),
        }
    }

    /// The periods of the surface's closed directions (u, v).
    pub(crate) fn periods(&self) -> [Option<f64>; 2] {
        match self {
            Surface::Cylinder { .. } | Surface::Cone { .. } | Surface::Sphere { .. } => [Some(TAU), None],
            Surface::Revolution { c, .. } => [Some(TAU), c.period()],
            Surface::Torus { .. } => [Some(TAU), Some(TAU)],
            Surface::Extrusion { c, .. } => [c.period(), None],
            Surface::Spline(s) => s.periods,
            Surface::Plane { .. } => [None, None],
        }
    }

    /// `(u, v)` of the surface's point nearest `q`, near `hint` where given (whole periods chosen
    /// to lie nearest it). A foot found by descent from `hint` that lies more than `miss` off `q`
    /// is sought again from the surface's coarse grid.
    pub(crate) fn invert(&self, q: Point3, hint: Option<(f64, f64)>, miss: f64) -> (f64, f64) {
        let hu = hint.map(|h| h.0);
        let hv = hint.map(|h| h.1);
        match self {
            Surface::Plane { f } => {
                let (lx, ly, _) = f.local(q);
                (lx, ly)
            }
            Surface::Cylinder { f, .. } => {
                let (lx, ly, lz) = f.local(q);
                (near_period(angle(ly, lx), hu), lz)
            }
            Surface::Cone { f, r, tan } => {
                let (lx, ly, lz) = f.local(q);
                // The point on the far side of the apex has its radius, and so its angle, negated.
                let a = if r + lz * tan < 0.0 { angle(-ly, -lx) } else { angle(ly, lx) };
                (near_period(a, hu), lz)
            }
            Surface::Sphere { f, .. } => {
                let (lx, ly, lz) = f.local(q);
                let rho = (lx * lx + ly * ly).sqrt();
                (near_period(angle(ly, lx), hu), lz.atan2(rho))
            }
            Surface::Torus { f, rmaj, rmin } => {
                let (lx, ly, lz) = f.local(q);
                let s = (lx * lx + ly * ly).sqrt();
                let a = angle(ly, lx);
                // The tube circle on q's side of the axis, or on a spindle torus the one across it
                // where that is nearer: its inner sheet (the lemon) reaches past the axis, and a
                // point there has no foot on its own side (bench singular/lemon: 4.0 off).
                let off = |rho: f64| ((rho * rho + lz * lz).sqrt() - rmin).abs();
                let (u, rho) = if rmaj < rmin && off(-s - rmaj) < off(s - rmaj) {
                    ((a + PI).rem_euclid(TAU), -s - rmaj)
                } else {
                    (a, s - rmaj)
                };
                let v = lz.atan2(rho);
                (near_period(u, hu), near_period(v.rem_euclid(TAU), hv))
            }
            _ => self.newton(q, hint, miss),
        }
    }

    /// Newton's method on the closest-point conditions, from `hint` or a coarse grid's best point.
    /// A descent from the hint that ends off `q` (stuck on a pole, where a partial vanishes, or
    /// pinned on a narrow patch's edge) is tried again from the grid, which is taken only where it
    /// lands on `q`; otherwise the hint's foot stands, and the walk refuses it (`piece`; bench
    /// integration/utah-teapot: a walk from a collapsed side's corner never left it;
    /// swept/tumbling-cube: one pinned to a strip's edge).
    fn newton(&self, q: Point3, hint: Option<(f64, f64)>, miss: f64) -> (f64, f64) {
        let local = self.descend(q, hint.unwrap_or_else(|| self.start(q)));
        let off = |(u, v): (f64, f64)| self.d1(u, v).0.distance(q);
        let (mut u, mut v) = match hint {
            Some(_) if off(local) > miss => {
                let global = self.descend(q, self.start(q));
                if off(global) <= miss { global } else { local }
            }
            _ => local,
        };
        // Whole periods chosen to lie nearest the hint, as the analytic inversions do.
        if let Some((hu, hv)) = hint {
            let near = |x: f64, h: f64, p: Option<f64>| match p {
                Some(p) if p > 0.0 => x + ((h - x) / p).round() * p,
                _ => x,
            };
            let periods = self.periods();
            u = near(u, hu, periods[0]);
            v = near(v, hv, periods[1]);
        }
        (u, v)
    }

    /// Newton's descent toward the point of the surface nearest `q`, from `(u, v)`. Each
    /// Gauss–Newton step is taken only where it brings the point nearer `q`, halved up to
    /// [`HALVINGS`] times until it does; where none does, the descent stops. Gauss–Newton leaves out
    /// the surface's curvature, and where a point lies a little off a surface with one partial
    /// nearly vanishing, full steps circle the foot without settling and the last lands anywhere on
    /// the circuit (bench swept/hypoid-gear-cut-by-its-pinion: an edge 2.8e-6 off its face,
    /// inverted to a foot 1.3e-4 off it). The point kept is so always the nearest seen.
    fn descend(&self, q: Point3, start: (f64, f64)) -> (f64, f64) {
        let ((u0, u1), (v0, v1)) = self.domain();
        let periods = self.periods();
        let scale = (u1 - u0).abs().max(v1 - v0).max(1.0);
        let settled = NEWTON_STEP * if scale.is_finite() { scale } else { 1.0 };
        // An open direction's parameter is kept in its domain. A closed direction's is kept within
        // one period of its start: at a pole, where the other partial vanishes, a step can throw it
        // billions of periods off, and its digits with it (bench geometry/ellipsoid: a whole
        // surface read as nearly none). The point is the same a whole number of periods on, and
        // the hint's period is chosen by the caller.
        let place = |mut u: f64, mut v: f64| {
            if periods[0].is_none() && u0.is_finite() {
                u = u.clamp(u0, u1);
            }
            if periods[1].is_none() && v0.is_finite() {
                v = v.clamp(v0, v1);
            }
            if let Some(p) = periods[0].filter(|p| *p > 0.0)
                && u0.is_finite()
            {
                u = u0 + (u - u0).rem_euclid(p);
            }
            if let Some(p) = periods[1].filter(|p| *p > 0.0)
                && v0.is_finite()
            {
                v = v0 + (v - v0).rem_euclid(p);
            }
            (u, v)
        };
        let (mut u, mut v) = start;
        let (mut p, mut su, mut sv) = self.d1(u, v);
        let mut off = p.distance(q);
        if !off.is_finite() {
            return start;
        }
        for _ in 0..60 {
            let r = q - p;
            // Gauss–Newton: [su·su su·sv; su·sv sv·sv] [du dv] = [r·su r·sv].
            let (a, b, c) = (su.dot(su), su.dot(sv), sv.dot(sv));
            let (e, g) = (r.dot(su), r.dot(sv));
            let det = a * c - b * b;
            let (du, dv) = if det.abs() > DEGENERATE * (a * c).max(f64::MIN_POSITIVE) {
                ((e * c - b * g) / det, (a * g - b * e) / det)
            } else if a > c {
                (if a > 0.0 { e / a } else { 0.0 }, 0.0)
            } else {
                (0.0, if c > 0.0 { g / c } else { 0.0 })
            };
            let mut step = 1.0;
            let mut taken = None;
            for _ in 0..=HALVINGS {
                let (nu, nv) = place(u + du * step, v + dv * step);
                if nu.is_finite() && nv.is_finite() {
                    let (np, nsu, nsv) = self.d1(nu, nv);
                    let noff = np.distance(q);
                    if noff < off {
                        taken = Some((nu, nv, np, nsu, nsv, noff));
                        break;
                    }
                }
                step *= 0.5;
            }
            let Some(next) = taken else { break };
            (u, v, p, su, sv, off) = next;
            if du.abs().max(dv.abs()) <= settled {
                break;
            }
        }
        (u, v)
    }

    /// A first guess at the parameters of `q`, for Newton's method.
    fn start(&self, q: Point3) -> (f64, f64) {
        match self {
            Surface::Extrusion { c, d } => {
                // The curve's point nearest `q` across the extrusion (its distance square to the
                // extrusion's direction), sought over a few points of each of the curve's pieces;
                // then the height. A curve off any one plane across the extrusion has no point
                // nearest `q` slid back onto such a plane (bench swept/coil-spring-pushed-aslant-
                // through-plate: a directrix of 470 spans rising 1.2 along a tilted extrusion,
                // inverted to a foot 4.7 off a point lying on it).
                let dd = d.dot(*d);
                if dd.is_nan() || dd <= 0.0 {
                    return (c.invert(q, None), 0.0);
                }
                let across = |t: f64| {
                    let r = q - c.d1(t).0;
                    (r - *d * (r.dot(*d) / dd)).length()
                };
                let (lo, hi) = c.domain();
                let u = match c.breaks(lo, hi) {
                    Ok(inner) if lo.is_finite() && hi.is_finite() => {
                        let ends: Vec<f64> = std::iter::once(lo).chain(inner).chain(std::iter::once(hi)).collect();
                        let mut best = (across(hi), hi);
                        for w in ends.windows(2) {
                            let &[a, b] = w else { continue };
                            for i in 0..4 {
                                let t = a + (b - a) * i as f64 / 4.0;
                                let e = across(t);
                                if e < best.0 {
                                    best = (e, t);
                                }
                            }
                        }
                        best.1
                    }
                    _ => c.invert(q - *d * ((q - c.d1(0.0).0).dot(*d) / dd), None),
                };
                (u, (q - c.d1(u).0).dot(*d) / dd)
            }
            Surface::Revolution { c, o, axis } => {
                // Match radius from the axis and height along it over a coarse run of the curve,
                // then the turn that carries that curve point to `q`.
                let polar = |p: Point3| {
                    let rel = p - *o;
                    let a = rel.dot(*axis);
                    let rr = rel - *axis * a;
                    (a, rr)
                };
                let (qa, qr) = polar(q);
                let (lo, hi) = c.domain();
                let (lo, hi) = if lo.is_finite() && hi.is_finite() { (lo, hi) } else { (-1.0, 1.0) };
                let mut best = (lo, f64::INFINITY);
                for i in 0..=64 {
                    let t = lo + (hi - lo) * i as f64 / 64.0;
                    let (a, rr) = polar(c.d1(t).0);
                    let d = (a - qa).powi(2) + (rr.length() - qr.length()).powi(2);
                    if d < best.1 {
                        best = (t, d);
                    }
                }
                let (_, rr) = polar(c.d1(best.0).0);
                let u = angle(axis.cross(rr).dot(qr), rr.dot(qr));
                (u, best.0)
            }
            _ => {
                let ((u0, u1), (v0, v1)) = self.domain();
                let fin = |a: f64, b: f64| if a.is_finite() && b.is_finite() { (a, b) } else { (-1.0, 1.0) };
                let ((u0, u1), (v0, v1)) = (fin(u0, u1), fin(v0, v1));
                let n = 24;
                let mut best = (u0, v0, f64::INFINITY);
                for i in 0..=n {
                    for j in 0..=n {
                        let (u, v) = (u0 + (u1 - u0) * i as f64 / n as f64, v0 + (v1 - v0) * j as f64 / n as f64);
                        let d = self.d1(u, v).0.distance(q);
                        if d < best.2 {
                            best = (u, v, d);
                        }
                    }
                }
                (best.0, best.1)
            }
        }
    }

    /// Parameters strictly between `lo` and `hi` along `axis` (0 = u, 1 = v) where the surface's
    /// smoothness may break or where a quadrature piece should end (see [`Curve::breaks`]).
    pub(crate) fn breaks(&self, axis: usize, lo: f64, hi: f64) -> Result<Vec<f64>> {
        match (self, axis) {
            (Surface::Plane { .. }, _) | (Surface::Cylinder { .. } | Surface::Cone { .. }, 1) => Ok(Vec::new()),
            (Surface::Cylinder { .. } | Surface::Cone { .. } | Surface::Sphere { .. } | Surface::Torus { .. }, _) => angular(lo, hi),
            (Surface::Revolution { .. }, 0) => angular(lo, hi),
            (Surface::Revolution { c, .. }, _) => c.breaks(lo, hi),
            (Surface::Extrusion { c, .. }, 0) => c.breaks(lo, hi),
            (Surface::Extrusion { .. }, _) => Ok(Vec::new()),
            (Surface::Spline(s), 0) => knots_between(&s.u.knots, s.u.domain(), s.periods[0], lo, hi),
            (Surface::Spline(s), _) => knots_between(&s.v.knots, s.v.domain(), s.periods[1], lo, hi),
        }
    }

    /// The parameter ranges (a periodic direction's one period; an extrusion's v unbounded).
    pub(crate) fn domain(&self) -> ((f64, f64), (f64, f64)) {
        let half = (-PI / 2.0, PI / 2.0);
        match self {
            Surface::Sphere { .. } => ((0.0, TAU), half),
            Surface::Torus { .. } => ((0.0, TAU), (0.0, TAU)),
            Surface::Cylinder { .. } | Surface::Cone { .. } => ((0.0, TAU), (f64::NEG_INFINITY, f64::INFINITY)),
            Surface::Revolution { c, .. } => ((0.0, TAU), c.domain()),
            Surface::Extrusion { c, .. } => (c.domain(), (f64::NEG_INFINITY, f64::INFINITY)),
            Surface::Plane { .. } => ((f64::NEG_INFINITY, f64::INFINITY), (f64::NEG_INFINITY, f64::INFINITY)),
            Surface::Spline(s) => s.domain(),
        }
    }
}

/// When the normal equations' determinant is this small against their diagonal, the point is on a
/// degenerate line (a pole) and only the better-conditioned parameter moves.
const DEGENERATE: f64 = 1e-12; // tol: relative conditioning of a 2×2 system

#[cfg(test)]
mod tests {
    use super::*;

    fn world() -> Frame {
        Frame { o: Point3::new(1.0, 2.0, 3.0), x: Vec3::new(1.0, 0.0, 0.0), y: Vec3::new(0.0, 1.0, 0.0), z: Vec3::new(0.0, 0.0, 1.0) }
    }

    #[test]
    fn a_spindle_torus_breaks_on_hostile_spans() {
        let t = Surface::Torus { f: world(), rmaj: 1.0, rmin: 2.0 };
        // Infinite and not-a-number ends, and ends so large a turn adds nothing: an answer, not a hang.
        for (lo, hi) in [(0.0, f64::INFINITY), (f64::NEG_INFINITY, 0.0), (f64::NAN, 1.0), (1e17, 1e17 + 4.0), (-1e300, 1e300)] {
            let _ = t.breaks(1, lo, hi);
        }
    }

    #[test]
    fn analytic_surfaces_invert_their_own_points() {
        let f = world();
        let surfaces = [
            Surface::Cylinder { f, r: 2.0 },
            Surface::Cone { f, r: 2.0, tan: 0.3 },
            Surface::Sphere { f, r: 2.0 },
            Surface::Torus { f, rmaj: 3.0, rmin: 1.0 },
            // A spindle torus, whose last two points lie on its inner sheet, past the axis.
            Surface::Torus { f, rmaj: 1.0, rmin: 2.0 },
        ];
        for s in &surfaces {
            for (u, v) in [(0.3, 0.2), (2.5, -0.7), (5.9, 0.9), (1.0, 2.5), (4.0, -2.2)] {
                let (p, _, _) = s.d1(u, v);
                let (iu, iv) = s.invert(p, Some((u, v)), 1e-12);
                let q = s.d1(iu, iv).0;
                assert!(q.distance(p) < 1e-12, "{s:?} at {u},{v}: {iu},{iv}");
            }
        }
    }

    #[test]
    fn far_runs_of_a_closed_spline_are_bounded() {
        // Far from zero the periods a run touches round apart from its span; with a tiny period
        // they pass i64. Each comes back promptly, refused or within the cap.
        let knots = [0.0, 0.0, 0.25, 0.5, 0.75, 1.0, 1.0];
        for (p, lo, hi) in [(1.0, 1e20, 1e20 + 16384.0), (0.1, 1e20, 1e20), (1e-300, 1e20, 1e20), (1e-3, -1e20, -1e20 + 1.0)] {
            if let Ok(cuts) = knots_between(&knots, (0.0, 1.0), Some(p), lo, hi) {
                assert!(cuts.len() as f64 <= MAX_PIECES, "{p} {lo} {hi}: {} cuts", cuts.len());
            }
        }
    }

    #[test]
    fn runs_needing_endless_pieces_are_refused() {
        // A closed spline with a vanishing period over an ordinary range, and a vast angle.
        let knots = [0.0, 0.0, 1e-300, 1e-300];
        assert!(knots_between(&knots, (0.0, 1e-300), Some(1e-300), 0.0, 1.0).is_err());
        assert!(angular(0.0, 1e12).is_err());
        assert!(knots_between(&knots, (0.0, 1e-300), Some(1e-300), 0.0, 1e-299).is_ok());
    }

    #[test]
    fn a_revolution_of_a_line_is_a_cylinder() {
        let c = Curve::Line { p: Point3::new(2.0, 0.0, 0.0), v: Vec3::new(0.0, 0.0, 1.0) };
        let s = Surface::Revolution { c, o: Point3::ORIGIN, axis: Vec3::new(0.0, 0.0, 1.0) };
        let (p, su, sv) = s.d1(PI / 2.0, 5.0);
        assert!(p.distance(Point3::new(0.0, 2.0, 5.0)) < 1e-14);
        assert!((su - Vec3::new(-2.0, 0.0, 0.0)).length() < 1e-14);
        assert!((sv - Vec3::new(0.0, 0.0, 1.0)).length() < 1e-14);
    }
}
