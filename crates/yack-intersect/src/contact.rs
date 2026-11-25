// yack: How two surfaces meet at a point already known to lie on both: whether they touch there only.
// Adapted from springrep@a03e8a1 src/boolean/contact.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from boolean/contact.rs to intersect/contact.rs (M2.1, T2); paths only; then to yack-intersect (M3 W4), paths and visibility only.
// M3 hardening: a spline patch's pole as one type shared by the touch and the turn, named steps,
// destructured pairs instead of indexing; the same arithmetic in the same order.
//! How two surfaces meet at a point already known to lie on both: whether they touch there only.
//!
//! Where the surfaces are tangent at the point, the gap between them is, to second order, the
//! quadratic form of the difference of their shape operators on the common tangent plane. When it
//! is definite the surfaces share the point alone: no intersection curve passes through it, and
//! within `reach` of it they lie within the tolerance of each other, so anything a trace finds there
//! is that point.

use super::csi;
use crate::{
    geom::{NurbsSurface, Surface},
    math::{Point3, Vec2, Vec3},
    tol,
};

/// A point where two surfaces touch and share nothing else nearby.
#[derive(Clone, Copy, Debug)]
pub struct Touch {
    pub p: Point3,
    /// How far from `p` the surfaces stay within the tolerance of each other.
    pub reach: f64,
}

/// Steps of a tenth of a micrometre for the normal's turn: the normals turn measurably, rounding
/// does not.
const TURN_STEP: f64 = 1e-4; // tol: a difference step, not a tolerance

/// A step of a micrometre along a surface: past the band of any surfaces that bend apart
/// measurably.
const GAP_STEP: f64 = 1e-3; // tol: a probing step, not a tolerance

/// A spline patch is at a pole where one partial derivative is shorter than this share of the other.
const POLE_RATIO: f64 = 1e-9; // tol: a relative length, for a derivative that vanishes

/// Two directions leaving a pole are too near parallel to solve along when the determinant of
/// their dot products is below this share of its largest.
const PARALLEL_RATIO: f64 = 1e-12; // tol: a relative determinant

/// The lines a pole's turn is taken along, as shares of the free parameter's range.
const TURN_LINES: [f64; 2] = [0.25, 0.75];

/// How many equal parts of the free parameter's range the lines probing a pole's gap split it into.
const GAP_LINES: u32 = 16;

/// A guard against rounding in the normals' differences, not a tolerance: an eigenvalue nearer
/// zero than this share of the other may be zero.
const ROUNDING: f64 = 1e-6; // tol: a relative eigenvalue guard

/// A pole of a spline patch: a parameter point where one parameter, the free one, moves the surface
/// point nowhere. The free parameter's lines leave it into the patch, stepping the other parameter
/// away from its nearer end.
struct Pole {
    uv: Vec2,
    free_u: bool,
    /// The sign of the other parameter's step into the patch.
    inward: f64,
    /// The free parameter's range.
    free: (f64, f64),
}

impl Pole {
    /// The pole of `n` at `uv`, where its partial derivatives are `su` and `sv`: the shorter one's
    /// parameter is free.
    fn new(n: &NurbsSurface, uv: Vec2, su: Vec3, sv: Vec3) -> Pole {
        let ((u0, u1), (v0, v1)) = n.domain();
        let free_u = su.norm() <= sv.norm();
        let (inward, free) = if free_u {
            (if uv.y - v0 > v1 - uv.y { -1.0 } else { 1.0 }, (u0, u1))
        } else {
            (if uv.x - u0 > u1 - uv.x { -1.0 } else { 1.0 }, (v0, v1))
        };
        Pole { uv, free_u, inward, free }
    }

    /// The pole of `s` at `p`, where `s` is a spline patch whose point at `p`'s parameters is `p`;
    /// `None` away from a pole.
    fn at(s: &Surface, p: Point3) -> Option<Pole> {
        let Surface::Nurbs(n) = s else { return None };
        let uv = s.project(p);
        let (q, su, sv) = s.d1(uv.x, uv.y);
        if q.dist(p) > tol::LINEAR
            || su.norm().min(sv.norm()) > POLE_RATIO * su.norm().max(sv.norm())
        {
            return None;
        }
        Some(Pole::new(n, uv, su, sv))
    }

    /// The free parameter's value a share `f` along its range.
    fn free_at(&self, f: f64) -> f64 {
        let (lo, hi) = self.free;
        lo + (hi - lo) * f
    }

    /// The line leaving the pole at the free parameter's value `f`: its start, and its unit step
    /// into the patch, in parameters.
    fn line(&self, f: f64) -> (Vec2, Vec2) {
        if self.free_u {
            (Vec2::new(f, self.uv.y), Vec2::new(0.0, self.inward))
        } else {
            (Vec2::new(self.uv.x, f), Vec2::new(self.inward, 0.0))
        }
    }
}

/// The change of `s`'s unit normal along the tangent directions `e`, by central differences of the
/// parameters; at a pole, where they do not move along the surface, by [`pole_turn`].
fn normal_turn(s: &Surface, p: Point3, e: [Vec3; 2], flip: bool) -> Option<[Vec3; 2]> {
    let uv = s.project(p);
    let (_, su, sv) = s.d1(uv.x, uv.y);
    let (g11, g12, g22) = (su.dot(su), su.dot(sv), sv.dot(sv));
    let det = g11 * g22 - g12 * g12;
    let sign = if flip { -1.0 } else { 1.0 };
    if det <= f64::EPSILON * g11 * g22 {
        let Surface::Nurbs(n) = s else { return None };
        let turn = pole_turn(s, &Pole::new(n, uv, su, sv), e)?;
        return Some(turn.map(|t| t * sign));
    }
    let turn = |d: Vec3| {
        let (bu, bv) = (su.dot(d) * TURN_STEP, sv.dot(d) * TURN_STEP);
        let step = Vec2::new((g22 * bu - g12 * bv) / det, (g11 * bv - g12 * bu) / det);
        let (a, b) = (uv + step, uv - step);
        (s.normal(a.x, a.y) - s.normal(b.x, b.y)) * (sign / (2.0 * TURN_STEP))
    };
    Some(e.map(turn))
}

/// The touch at `p`, a pole of `s` on `other`: a surface need not curve as a quadratic form at its
/// pole (a patch closing on one point is curved by its own section in each half plane), so the gap
/// is measured along each line leaving the pole into the patch, a step from it. They touch only
/// there when every gap lies on one side, and stay within the tolerance out to the reach of the
/// least curved line.
fn pole_touch(s: &Surface, other: &Surface, p: Point3, pole: &Pole) -> Option<Touch> {
    let uv = other.project(p);
    let n_other = other.normal(uv.x, uv.y);
    let su = s.project(p);
    if s.normal(su.x, su.y).cross(n_other).norm() > tol::TANGENT {
        return None;
    }
    let mut least = f64::INFINITY;
    let mut sign = 0.0;
    for k in 0..=GAP_LINES {
        let (at, inward) = pole.line(pole.free_at(f64::from(k) / f64::from(GAP_LINES)));
        let (_, au, av) = s.d1(at.x, at.y);
        let speed = (au * inward.x + av * inward.y).norm();
        if speed == 0.0 {
            return None;
        }
        let w = at + inward * (GAP_STEP / speed);
        let q = s.eval(w.x, w.y);
        let gap = csi::signed_distance(other, q);
        let d = (q - p).norm();
        if gap.abs() <= f64::EPSILON * d || sign * gap < 0.0 {
            return None;
        }
        sign = gap.signum();
        least = least.min(2.0 * gap.abs() / (d * d));
    }
    Some(Touch { p, reach: (2.0 * tol::LINEAR / least).sqrt() })
}

/// The normal's turn along `e` at a pole of a spline surface. A patch may meet its pole in a
/// wedge, so no step crosses it: the turn is taken one-sided along two of the lines leaving the
/// pole into the patch, and being linear in the direction, is solved for along `e` from those two.
fn pole_turn(s: &Surface, pole: &Pole, e: [Vec3; 2]) -> Option<[Vec3; 2]> {
    let n0 = s.normal(pole.uv.x, pole.uv.y);
    // The line leaving the pole at the free parameter's value `f`: its direction, and its point at
    // arc length `h`, to first order, as parameters.
    let leave = |f: f64, h: f64| -> Option<(Vec3, Vec2)> {
        let (at, inward) = pole.line(f);
        let (_, au, av) = s.d1(at.x, at.y);
        let along = au * inward.x + av * inward.y;
        let speed = along.norm();
        (speed > 0.0).then(|| (along / speed, at + inward * (h / speed)))
    };
    // The direction of the line at share `f` of the free range, and the normal's turn along it.
    let turn_along = |f: f64| -> Option<(Vec3, Vec3)> {
        let f = pole.free_at(f);
        let (d, a) = leave(f, TURN_STEP)?;
        let (_, b) = leave(f, 2.0 * TURN_STEP)?;
        let turn = (s.normal(a.x, a.y) * 4.0 - s.normal(b.x, b.y) - n0 * 3.0) / (2.0 * TURN_STEP);
        Some((d, turn))
    };
    let [first, second] = TURN_LINES;
    let (da, ta) = turn_along(first)?;
    let (db, tb) = turn_along(second)?;
    let (aa, ab, bb) = (da.dot(da), da.dot(db), db.dot(db));
    let det = aa * bb - ab * ab;
    if det <= PARALLEL_RATIO * aa * bb {
        return None;
    }
    let along = |w: Vec3| {
        let (ra, rb) = (da.dot(w), db.dot(w));
        let (x, y) = ((bb * ra - ab * rb) / det, (aa * rb - ab * ra) / det);
        ta * x + tb * y
    };
    Some(e.map(along))
}

/// The gap form at `p` between tangent surfaces: the difference of their normals' turn along the
/// tangent plane, `b`'s normal taken the way `a`'s points. `a`'s height over `b` along `a`'s normal
/// is `−½ wᵀMw` for a step `w`.
struct GapForm {
    /// The form's eigenvalues, least first.
    eigen: (f64, f64),
    /// `a`'s and `b`'s normals at the point.
    normals: (Vec3, Vec3),
}

impl GapForm {
    fn at(a: &Surface, b: &Surface, p: Point3) -> Option<GapForm> {
        let (ua, ub) = (a.project(p), b.project(p));
        let (na, nb) = (a.normal(ua.x, ua.y), b.normal(ub.x, ub.y));
        if na.cross(nb).norm() > tol::TANGENT {
            return None;
        }
        let e0 = na.any_perpendicular().unit()?;
        let e = [e0, na.cross(e0)];
        let [a0, a1] = normal_turn(a, p, e, false)?;
        let [b0, b1] = normal_turn(b, p, e, na.dot(nb) < 0.0)?;
        let [e0, e1] = e;
        let (d0, d1) = (a0 - b0, a1 - b1);
        let (m11, m22, m12) = (d0.dot(e0), d1.dot(e1), 0.5 * (d0.dot(e1) + d1.dot(e0)));
        let mean = 0.5 * (m11 + m22);
        let spread = (0.25 * (m11 - m22) * (m11 - m22) + m12 * m12).sqrt();
        Some(GapForm { eigen: (mean - spread, mean + spread), normals: (na, nb) })
    }
}

/// How far either side of `q`, a point of both surfaces on a curve along `dir` where they are
/// tangent, they stay within the tolerance of each other: `√(2·tol/κ)` for the curvature `κ` of
/// the gap across the curve, measured a step off it each way; zero where they are not tangent.
pub fn band(a: &Surface, b: &Surface, q: Point3, dir: Vec3) -> f64 {
    let (ua, ub) = (a.project(q), b.project(q));
    let (na, nb) = (a.normal(ua.x, ua.y), b.normal(ub.x, ub.y));
    if na.cross(nb).norm() > tol::TANGENT {
        return 0.0;
    }
    let Some(across) = na.cross(dir).unit() else {
        return 0.0;
    };
    let gap = [1.0, -1.0]
        .into_iter()
        .map(|side: f64| {
            let w = a.project(q + across * (side * GAP_STEP));
            csi::signed_distance(b, a.eval(w.x, w.y)).abs()
        })
        .fold(0.0, f64::max);
    if gap <= tol::LINEAR {
        return GAP_STEP;
    }
    (2.0 * tol::LINEAR / (2.0 * gap / (GAP_STEP * GAP_STEP))).sqrt()
}

/// Which side of `b` the surface `a` lies on near `p`, a point of both where they touch without
/// crossing: `Some(true)` where `a` bends away along `b`'s normal. `None` where the gap form
/// changes sign (they cross) or vanishes (they osculate, or coincide).
pub fn side_of(a: &Surface, b: &Surface, p: Point3) -> Option<bool> {
    let GapForm { eigen: (l1, l2), normals: (na, nb) } = GapForm::at(a, b, p)?;
    let big = l1.abs().max(l2.abs());
    if big == 0.0 || !big.is_finite() {
        return None;
    }
    let guard = ROUNDING * big;
    // A semidefinite form is a touch along the null direction's curve; off it, the other eigenvalue
    // alone gives the side.
    let below = if l1 >= -guard && l2 > guard {
        true
    } else if l2 <= guard && l1 < -guard {
        false
    } else {
        return None;
    };
    Some(below != (na.dot(nb) > 0.0))
}

/// The touch at `p`, which lies on both surfaces: `Some` where their normals are parallel within
/// [`tol::TANGENT`] and their gap is a definite quadratic form.
pub fn touch_at(a: &Surface, b: &Surface, p: Point3) -> Option<Touch> {
    for (s, other) in [(a, b), (b, a)] {
        if let Some(pole) = Pole::at(s, p) {
            return pole_touch(s, other, p, &pole);
        }
    }
    let (l1, l2) = GapForm::at(a, b, p)?.eigen;
    // Nearer zero than the guard the form may be only semidefinite, the surfaces touching along a
    // curve.
    let small = l1.abs().min(l2.abs());
    if l1 * l2 <= 0.0 || small <= ROUNDING * l1.abs().max(l2.abs()) {
        return None;
    }
    Some(Touch { p, reach: (2.0 * tol::LINEAR / small).sqrt() })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::{Cylinder, Plane, Sphere};
    use crate::math::Frame;

    /// The octant of the sphere of radius `r` about the origin, as the rational biquadratic of
    /// Piegl and Tiller. A copy of yack-geom's test helper, whose cfg(test) items another crate's
    /// tests cannot see (M3 W3).
    fn octant(r: f64) -> Surface {
        let w = std::f64::consts::FRAC_1_SQRT_2;
        let equator = [(r, 0.0, 1.0), (r, r, w), (0.0, r, 1.0)];
        let mut ctrl = Vec::new();
        let mut weights = Vec::new();
        for &(x, y, wu) in &equator {
            for (j, wv) in [1.0, w, 1.0].into_iter().enumerate() {
                ctrl.push(match j {
                    0 => Vec3::new(x, y, 0.0),
                    1 => Vec3::new(x, y, r),
                    _ => Vec3::new(0.0, 0.0, r),
                });
                weights.push(wu * wv);
            }
        }
        let k = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
        let n = NurbsSurface::new((2, 2), (k.clone(), k), (3, 3), ctrl, Some(weights)).unwrap();
        Surface::Nurbs(Box::new(n))
    }

    fn cyl(o: [f64; 3], z: [f64; 3], r: f64) -> Surface {
        let frame = Frame::from_axis(Vec3::from_array(o), Vec3::from_array(z)).unwrap();
        Surface::Cylinder(Cylinder { frame, radius: r })
    }

    fn plane(o: Point3, z: Vec3) -> Surface {
        Surface::Plane(Plane { frame: Frame::from_axis(o, z).unwrap() })
    }

    #[test]
    fn a_plane_on_a_spline_sphere_s_pole_touches_it_there() {
        // The plane z = 2 meets the sphere of radius 2 at its pole, where the octant's parameters
        // do not move: the gap is (x² + y²)/4, so they stay within the tolerance for √(4·tol) mm.
        let sphere = octant(2.0);
        let top = Vec3::new(0.0, 0.0, 2.0);
        let t = touch_at(&plane(top, Vec3::Z), &sphere, top).expect("a point touch");
        let want = (4.0 * tol::LINEAR).sqrt();
        assert!((t.reach - want).abs() <= 1e-4 * want, "{}", t.reach);
    }

    #[test]
    fn crossed_cylinders_bulging_apart_touch_at_a_point() {
        // Radius 1 about x below z = 0 and radius 4 about y above it: the gap is x²/8 + y²/2, so
        // its smaller curvature is 1/4 and they stay within 1e-7 for √(2e-7 · 4) mm.
        let a = cyl([0.0, 0.0, -1.0], [1.0, 0.0, 0.0], 1.0);
        let b = cyl([0.0, 0.0, 4.0], [0.0, 1.0, 0.0], 4.0);
        let t = touch_at(&a, &b, Point3::ZERO).expect("a point touch");
        assert!((t.reach - (8e-7f64).sqrt()).abs() <= 1e-6 * t.reach, "{}", t.reach);
    }

    #[test]
    fn nested_cylinders_crossing_at_a_tangent_point_are_not_a_point_touch() {
        // Radius 1 about x and radius 4 about y, both below z = 0: the gap x²/8 − y²/2 changes
        // sign, so two curves cross at the point.
        let a = cyl([0.0, 0.0, -1.0], [1.0, 0.0, 0.0], 1.0);
        let b = cyl([0.0, 0.0, -4.0], [0.0, 1.0, 0.0], 4.0);
        assert!(touch_at(&a, &b, Point3::ZERO).is_none());
    }

    #[test]
    fn a_plane_on_a_cylinder_touches_along_a_line_not_a_point() {
        let a = cyl([0.0, 0.0, -1.0], [1.0, 0.0, 0.0], 1.0);
        assert!(touch_at(&a, &plane(Point3::ZERO, Vec3::Z), Point3::ZERO).is_none());
    }

    #[test]
    fn a_ball_on_a_plane_touches_at_a_point_and_crossing_surfaces_do_not() {
        // Its axis along x, so the touching point lies on its equator, not at a pole.
        let ball = Surface::Sphere(Sphere {
            frame: Frame::from_axis(Vec3::new(0.0, 0.0, 2.0), Vec3::X).unwrap(),
            radius: 2.0,
        });
        let floor = plane(Point3::ZERO, Vec3::Z);
        let t = touch_at(&ball, &floor, Point3::ZERO).expect("a point touch");
        assert!((t.reach - (4e-7f64).sqrt()).abs() <= 1e-6 * t.reach);
        assert!(touch_at(&ball, &plane(Point3::ZERO, Vec3::X), Point3::ZERO).is_none());
    }
}
