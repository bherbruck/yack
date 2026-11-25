// yack: the nearest point of a spline surface: Newton's search from a seed or a hint, and poles.
// Adapted from springrep@a03e8a1 src/geom/nurbs.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/geom/nurbs.rs to yack-geom (M3 W3); split out of
// nurbs.rs into its own module (M3 hardening). M3 hardening: the parameter that moves nothing at
// a pole is a `Parameter` instead of an index into pairs, and named constants; the same
// arithmetic in the same order.
//! The nearest point of a spline surface: Newton's search from a seed or a hint, and poles.

use super::{NEAR_SHARE, NurbsSurface, STALLED};
use crate::{
    math::{Point3, Vec2, Vec3},
    numeric::solve2,
    tol,
};

/// The most Newton steps a search takes.
const SEARCH_STEPS: usize = 60;
/// A step that moves the point less than this share of [`tol::LINEAR`] moves it by rounding alone.
const ROUNDING_STEP: f64 = 1e-6; // tol: share of the linear tolerance
/// A step that leaves the point farther than this relative rounding is no nearer.
const FARTHER: f64 = 1e-12; // tol: relative rounding of a distance
/// The absolute rounding of that comparison, for a point already on the surface.
const DISTANCE_ROUNDING: f64 = 1e-15; // tol: absolute rounding of a distance
/// Parameters that move less than this, relative to their size, have settled.
const PARAMETER_ROUNDING: f64 = 1e-15; // tol: relative rounding of a parameter
/// The least share of a Newton step a search tries before it stops.
const LEAST_SHARE: f64 = 1e-3; // tol: a share of a step, not a length
/// A first derivative this small beside the other is a pole's.
const POLE_RATIO: f64 = 1e-9; // tol: relative size of a derivative
/// The golden-section and Newton steps along the line out of a pole.
const POLE_STEPS: usize = 40;

/// One of a surface's two parameters.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Parameter {
    U,
    V,
}

impl NurbsSurface {
    /// Newton steps towards the surface point nearest `p` from `uv`, held inside the domain in a
    /// parameter that is not periodic, given the second derivatives at `uv` when the caller has
    /// them. Each step is the full Newton step of the squared distance where its Hessian is
    /// positive definite, which converges quadratically even off the surface, and the
    /// Gauss–Newton step elsewhere.
    ///
    /// Returns the parameters, their point's distance from `p`, and whether the search settled on
    /// a foot of `p`: a point from which no way along the surface inside the domain comes nearer.
    /// A search that stops short, where no fraction of its step comes nearer, has found a point
    /// and not a foot.
    pub(super) fn search(
        &self,
        p: Point3,
        mut uv: Vec2,
        start: Option<[Vec3; 6]>,
    ) -> (Vec2, f64, bool) {
        let ((u0, u1), (v0, v1)) = self.domain();
        let clamp = |uv: Vec2| {
            Vec2::new(
                if self.u_period.is_some() { uv.x } else { uv.x.clamp(u0, u1) },
                if self.v_period.is_some() { uv.y } else { uv.y.clamp(v0, v1) },
            )
        };
        let inside = clamp(uv);
        let mut at = match start {
            Some(at) if inside == uv => at,
            _ => self.d2(inside.x, inside.y),
        };
        uv = inside;
        // At a pole one parameter moves nothing, and no step solved there says which of the lines
        // out of it leads to the foot: the line is chosen first, and the search starts down it.
        if let Some(free) = pole_of(&at) {
            match self.off_pole(p, uv, free) {
                Some(off) => {
                    uv = off;
                    at = self.d2(uv.x, uv.y);
                }
                None => return (self.wrapped(uv), at[0].dist(p), true),
            }
        }
        for _ in 0..SEARCH_STEPS {
            let [q, su, sv, suu, suv, svv] = at;
            let r = q - p;
            let (g0, g1) = (su.dot(r), sv.dot(r));
            let (a, b, d) = (su.dot(su), su.dot(sv), sv.dot(sv));
            let (ha, hb, hd) = (a + suu.dot(r), b + suv.dot(r), d + svv.dot(r));
            // A parameter at an end of its range, with the nearer point beyond it, stays there: the
            // foot is on that boundary, and the search goes on along it alone. A step solved in
            // both and then cut back to the range is not the nearest point of the boundary.
            let held = |x: f64, lo: f64, hi: f64, g: f64, free: bool| {
                !free && ((x <= lo && g > 0.0) || (x >= hi && g < 0.0))
            };
            let hold_u = held(uv.x, u0, u1, g0, self.u_period.is_some());
            let hold_v = held(uv.y, v0, v1, g1, self.v_period.is_some());
            let one = |g: f64, h: f64, gauss: f64| {
                let by = if h > 0.0 { h } else { gauss };
                (by > 0.0).then(|| -g / by)
            };
            let step = match (hold_u, hold_v) {
                // A corner of the domain, the nearer points beyond both its sides.
                (true, true) => return (self.wrapped(uv), r.norm(), true),
                (true, false) => one(g1, hd, d).map(|dv| (0.0, dv)),
                (false, true) => one(g0, ha, a).map(|du| (du, 0.0)),
                (false, false) => (ha > 0.0 && ha * hd - hb * hb > 0.0)
                    .then(|| solve2(ha, hb, hb, hd, -g0, -g1))
                    .flatten()
                    .or_else(|| solve2(a, b, b, d, -g0, -g1)),
            };
            let Some((du, dv)) = step else {
                break;
            };
            // A step this short moves the point by rounding alone: take it and stop, rather than
            // evaluate the surface again to confirm it.
            if (su * du + sv * dv).norm() <= ROUNDING_STEP * tol::LINEAR {
                uv = clamp(Vec2::new(uv.x + du, uv.y + dv));
                return (self.wrapped(uv), self.eval(uv.x, uv.y).dist(p), true);
            }
            // A full step from far off can overshoot on a strongly curved or twisted patch; halve
            // it until it gets nearer, or stop where no fraction of it does.
            let mut f = 1.0;
            let next = loop {
                let next = clamp(Vec2::new(uv.x + f * du, uv.y + f * dv));
                let there = self.d2(next.x, next.y);
                if there[0].dist(p) <= r.norm() * (1.0 + FARTHER) + DISTANCE_ROUNDING {
                    break Some((next, there));
                }
                f *= 0.5;
                if f < LEAST_SHARE {
                    break None;
                }
            };
            // No fraction of a step this short gets nearer by more than the rounding of measuring
            // it: the foot is here, as near as a step that short (bench swept/ring-carried-along-
            // a-spline: one in sixteen local searches stopped at the foot unsettled, and each one
            // then searched the whole surface again).
            let Some((next, there)) = next else {
                // Only at a stationary point: the distance's gradient along the surface at
                // rounding, not a stall short of the foot.
                let along = g0.abs() / su.norm().max(f64::MIN_POSITIVE)
                    + g1.abs() / sv.norm().max(f64::MIN_POSITIVE);
                let settled = (su * du + sv * dv).norm() <= STALLED && along <= STALLED;
                return (self.wrapped(uv), r.norm(), settled);
            };
            let moved = (next.x - uv.x).abs() + (next.y - uv.y).abs();
            uv = next;
            at = there;
            if moved <= PARAMETER_ROUNDING * (1.0 + uv.x.abs() + uv.y.abs()) {
                // Parameters that move by rounding alone: settled where the gradient is rounding too.
                let along = g0.abs() / su.norm().max(f64::MIN_POSITIVE)
                    + g1.abs() / sv.norm().max(f64::MIN_POSITIVE);
                return (self.wrapped(uv), at[0].dist(p), along <= STALLED);
            }
        }
        (self.wrapped(uv), at[0].dist(p), false)
    }

    /// Where a search from the pole at `uv` starts: a point a way down the line out of the pole
    /// that heads for `p` most directly, at the nearest point of that line to `p`. `free` is the
    /// parameter that moves nothing at the pole; the other is at an end of its
    /// range there. `None` where every line out of the pole leads away from `p`: the pole is the
    /// foot.
    fn off_pole(&self, p: Point3, uv: Vec2, free: Parameter) -> Option<Vec2> {
        let ((u0, u1), (v0, v1)) = self.domain();
        // The free parameter's range, the other's, and where the other is pinned at the pole.
        let ((lo, hi), (w0, w1), pinned) = match free {
            Parameter::U => ((u0, u1), (v0, v1), uv.y),
            Parameter::V => ((v0, v1), (u0, u1), uv.x),
        };
        // Into the domain from the end the pole is on; a pole inside a range has no such way.
        let inward = if pinned <= w0 {
            1.0
        } else if pinned >= w1 {
            -1.0
        } else {
            return Some(uv);
        };
        let at = |t: f64, w: f64| match free {
            Parameter::U => Vec2::new(t, w),
            Parameter::V => Vec2::new(w, t),
        };
        // How fast the distance to `p` falls on the way in along the line at `t`.
        let fall = |t: f64| {
            let x = at(t, pinned);
            let [q, su, sv, ..] = self.d2(x.x, x.y);
            let line = match free {
                Parameter::U => sv,
                Parameter::V => su,
            };
            let n = line.norm();
            if n > 0.0 { -inward * line.dot(q - p) / n } else { f64::NEG_INFINITY }
        };
        // The best of the lines a grid names, then the best between its neighbours: the fall is
        // one smooth wave round a pole, so its peak lies between the grid's best and one of them.
        const LINES: usize = 16;
        let grid = |k: usize| lo + (hi - lo) * k as f64 / LINES as f64;
        let best = (0..=LINES).map(|k| (fall(grid(k)), k)).max_by(|x, y| x.0.total_cmp(&y.0))?;
        let (mut a, mut b) = (grid(best.1.saturating_sub(1)), grid((best.1 + 1).min(LINES)));
        let golden = 0.5 * (5f64.sqrt() - 1.0);
        let (mut c, mut d) = (b - golden * (b - a), a + golden * (b - a));
        let (mut fc, mut fd) = (fall(c), fall(d));
        for _ in 0..POLE_STEPS {
            if fc > fd {
                (b, d, fd) = (d, c, fc);
                c = b - golden * (b - a);
                fc = fall(c);
            } else {
                (a, c, fc) = (c, d, fd);
                d = a + golden * (b - a);
                fd = fall(d);
            }
        }
        let t = 0.5 * (a + b);
        if fall(t) <= 0.0 {
            return None;
        }
        // The nearest point of that line to `p`: Newton in the one parameter, from the pole.
        let mut w = pinned;
        for _ in 0..POLE_STEPS {
            let x = at(t, w);
            let [q, su, sv, suu, _, svv] = self.d2(x.x, x.y);
            let (line, bend) = match free {
                Parameter::U => (sv, svv),
                Parameter::V => (su, suu),
            };
            let r = q - p;
            let (g, gauss) = (line.dot(r), line.dot(line));
            let h = gauss + bend.dot(r);
            let by = if h > 0.0 { h } else { gauss };
            if by <= 0.0 {
                break;
            }
            let mut step = -g / by;
            let mut next = (w + step).clamp(w0, w1);
            // No further than gets nearer, as the search over both parameters does.
            while self.eval(at(t, next).x, at(t, next).y).dist(p) > r.norm() * (1.0 + FARTHER)
                && step.abs() > PARAMETER_ROUNDING * (w1 - w0)
            {
                step *= 0.5;
                next = (w + step).clamp(w0, w1);
            }
            let moved = (next - w).abs();
            w = next;
            if moved <= PARAMETER_ROUNDING * (1.0 + w.abs()) {
                break;
            }
        }
        // Still on the pole: the line fell at first and no step down it came nearer.
        if w == pinned {
            return None;
        }
        Some(at(t, w))
    }

    fn wrapped(&self, uv: Vec2) -> Vec2 {
        Vec2::new(self.bu.wrap(uv.x, self.u_period), self.bv.wrap(uv.y, self.v_period))
    }

    /// How near the surface a point must lie for [`NurbsSurface::project_near`] to take the point
    /// Newton finds from its hint as the nearest.
    pub fn near_radius(&self) -> f64 {
        tol::LINEAR.max(NEAR_SHARE * self.seed_grid().spacing)
    }

    /// The parameters of the surface point nearest `p`, found from `hint` when the surface passes
    /// within tolerance there, globally otherwise.
    pub fn project_near(&self, p: Point3, hint: Vec2) -> Vec2 {
        // A hint on a pole says nothing of the free parameter: Newton from it finds the point
        // nearest along that one line out of the pole, which can be a boundary's, not the patch's.
        let at = self.d2(hint.x, hint.y);
        let [_, su, sv, ..] = at;
        if su.norm().min(sv.norm()) <= POLE_RATIO * su.norm().max(sv.norm()) {
            return self.project(p);
        }
        let (local, d, settled) = self.search(p, hint, Some(at));
        // Global projection trusts the seed grid to resolve the surface: no two of its sheets come
        // within a fraction of the seed spacing of one point. A foot that near the surface is
        // therefore the nearest.
        if settled && d <= self.near_radius() {
            return local;
        }
        let global = self.project(p);
        if self.eval(global.x, global.y).dist(p) < d { global } else { local }
    }
}

/// The parameter that moves nothing at a point with these derivatives: a pole, where a whole row
/// of the surface is one point.
fn pole_of(at: &[Vec3; 6]) -> Option<Parameter> {
    let (a, d) = (at[1].norm(), at[2].norm());
    if a <= POLE_RATIO * d {
        Some(Parameter::U)
    } else if d <= POLE_RATIO * a {
        Some(Parameter::V)
    } else {
        None
    }
}
