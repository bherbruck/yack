// yack: springrep module src/topo/pcurve.rs
// Adapted from springrep@a03e8a1 src/topo/pcurve.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: none (verbatim); this header added.
use crate::{
    geom::{Curve, Surface},
    math::{Point3, Vec2, Vec3},
    tol,
};

/// Where a coedge runs in its face's parameter plane, as a function of its edge's parameter `t`.
#[derive(Clone, Debug, PartialEq)]
pub enum PCurve {
    /// `origin + t·dir`: every edge a sweep makes along or across its surface's parameter lines.
    Affine { origin: Vec2, dir: Vec2 },
    /// The edge's point projected onto the surface. A periodic parameter continues from `seed`,
    /// the coedge's parameters where it starts, so a seam's two coedges stay apart. Where the edge
    /// meets a pole, `u` is the one its direction there leaves the pole by.
    Projected { seed: Vec2 },
}

/// `curve(t)` projected onto `s` near `hint`. At a pole, where every `u` is the same point, `u` is
/// the angle the curve leaves the pole at towards `toward`, a parameter beside `t`: the limit of
/// its neighbours' `u`, which every caller finds alike whatever its sampling.
pub(crate) fn project_on(s: &Surface, curve: &Curve, t: f64, toward: f64, hint: Vec2) -> Vec2 {
    let (p, dp, ddp) = curve.d2(t);
    let uv = s.project_near(p, hint);
    let away = dp * (toward - t).signum();
    // The way the curve bends: its second derivative less its part along the curve.
    let bend = dp.unit().map(|d| ddp - d * ddp.dot(d));
    match pole_param(s, p, uv, away, bend, hint, false) {
        Some(u) => Vec2::new(u, uv.y),
        None => match pole_param(s, p, uv, away, None, hint, true) {
            Some(v) => Vec2::new(uv.x, v),
            None => uv,
        },
    }
}

/// `p` projected onto `s` near `hint`, a point of an edge between its ends. Where it lands on a
/// pole along which `v` is free (an imported edge that misses its face by its tolerance can run
/// past one), `v` stays at the hint's, as the analytic surfaces keep a free `u`.
pub(crate) fn project_past(s: &Surface, p: Point3, hint: Vec2) -> Vec2 {
    let uv = s.project_near(p, hint);
    let (_, _, sv) = s.d1(uv.x, uv.y);
    if sv.norm() <= tol::LINEAR {
        Vec2::new(uv.x, hint.y)
    } else {
        uv
    }
}

/// [`project_past`] of `curve(t1)`, continued from `hint` at `curve(t0)`. A periodic parameter
/// continues to the image nearest the last one, which is the curve's own only while each step turns
/// it by well under half a period; beside a pole a short step can turn it by nearly half, either
/// way, so such a step is halved until each part turns it by under a quarter.
pub(crate) fn follow(s: &Surface, curve: &Curve, (t0, t1): (f64, f64), hint: Vec2) -> Vec2 {
    let uv = project_past(s, curve.eval(t1), hint);
    let jumps = |per: Option<f64>, d: f64| per.is_some_and(|p| d.abs() > 0.25 * p);
    let turned = jumps(s.u_period(), uv.x - hint.x) || jumps(s.v_period(), uv.y - hint.y);
    let mid = 0.5 * (t0 + t1);
    if !turned || mid == t0 || mid == t1 {
        return uv;
    }
    let h = follow(s, curve, (t0, mid), hint);
    follow(s, curve, (mid, t1), h)
}

/// At a pole of `s` where `u` (or, with `in_v`, `v`) is free, the value of it at which the
/// surface leaves the pole along `away`; `None` away from such a pole. A curve leaving along the
/// surface's axis, as every curve through the pole of a torus whose tube touches its axis does,
/// leaves at the angle it bends toward, `bend`.
fn pole_param(
    s: &Surface,
    p: Point3,
    uv: Vec2,
    away: Vec3,
    bend: Option<Vec3>,
    hint: Vec2,
    in_v: bool,
) -> Option<f64> {
    let (_, su, sv) = s.d1(uv.x, uv.y);
    let d = away.unit_or_zero();
    if (if in_v { sv } else { su }).norm() > tol::LINEAR || d.norm() == 0.0 {
        return None;
    }
    let (x, period, near) = if in_v {
        let v = leaving_v(s, uv.x, d).unwrap_or_else(|| s.project(p + d * 1e-3).y);
        (v, s.v_period(), hint.y)
    } else {
        // Round a pole of an analytic surface any step along a direction in the tangent plane has
        // the same angle about it; a spline's pole need not be round, and a point beside it
        // projects onto a line leaving it at another angle.
        let u = match s {
            Surface::Nurbs(n) => leaving_u(n, uv.y, d),
            _ => None,
        };
        let axis = match s {
            Surface::Sphere(x) => Some(x.frame.z),
            Surface::Cone(x) => Some(x.frame.z),
            Surface::Torus(x) => Some(x.frame.z),
            _ => None,
        };
        let across = |v: Vec3, z: Vec3| v - z * v.dot(z);
        let step = match (axis, bend) {
            (Some(z), Some(k)) if across(d, z).norm() <= 1e-6 => across(k, z).unit().unwrap_or(d),
            _ => d,
        };
        let u = u.unwrap_or_else(|| s.project(p + step * 1e-3).x);
        (u, s.u_period(), hint.x)
    };
    Some(match period {
        Some(per) => x + ((near - x) / per).round() * per,
        None => x,
    })
}

/// At a pole of `s` along the line `u = u0`, where `v` is free, the `v` whose `u`-line leaves the
/// pole along `away`, by the surface's own derivative rather than by projecting a point near the
/// pole, which on a thin patch lands at any `v`. `None` where `v` has no known range.
fn leaving_v(s: &Surface, u0: f64, away: Vec3) -> Option<f64> {
    let (lo, hi) = match (s, s.v_period()) {
        (Surface::Nurbs(n), _) => n.domain().1,
        (_, Some(per)) => (0.0, per),
        _ => return None,
    };
    let inward = match s {
        Surface::Nurbs(n) if u0 > 0.5 * (n.domain().0.0 + n.domain().0.1) => -1.0,
        _ => 1.0,
    };
    let leaving = |v: f64| (s.d1(u0, v).1 * inward).unit();
    best_of(&leaving, away, (lo, hi))
}

/// At a pole of `n` along the line `v = v0`, where `u` is free, the `u` whose `v`-line leaves the
/// pole along `away`, by the surface's own derivative.
fn leaving_u(n: &crate::geom::NurbsSurface, v0: f64, away: Vec3) -> Option<f64> {
    let ((lo, hi), (v_lo, v_hi)) = n.domain();
    let inward = if v0 > 0.5 * (v_lo + v_hi) { -1.0 } else { 1.0 };
    let leaving = |u: f64| (n.d1(u, v0).2 * inward).unit();
    best_of(&leaving, away, (lo, hi))
}

/// Where `leaving` points along `away`, on `[lo, hi]`: the nearest of a run of samples, refined
/// round it.
fn best_of(leaving: &dyn Fn(f64) -> Option<Vec3>, away: Vec3, (lo, hi): (f64, f64)) -> Option<f64> {
    const SAMPLES: usize = 128;
    let cos = |x: f64| leaving(x).map_or(-2.0, |l| l.dot(away));
    let at = |k: usize| lo + (hi - lo) * k as f64 / SAMPLES as f64;
    let best = (0..=SAMPLES).max_by(|&a, &b| cos(at(a)).total_cmp(&cos(at(b))))?;
    // The cosine is flat at its peak, so rounding of 1e-16 moves the peak by 1e-8. The sine
    // between the directions, signed about the pole's normal, crosses zero there at full slope.
    for (j, k) in [
        (best.saturating_sub(1), best),
        (best, (best + 1).min(SAMPLES)),
    ] {
        let (mut a, mut b) = (at(j), at(k));
        let (Some(la), Some(lb)) = (leaving(a), leaving(b)) else {
            continue;
        };
        let Some(normal) = la.cross(lb).unit() else {
            continue;
        };
        let sine = |x: f64| leaving(x).map_or(0.0, |l| l.cross(away).dot(normal));
        let (sa, sb) = (sine(a), sine(b));
        for (x, s) in [(a, sa), (b, sb)] {
            if s.abs() <= 1e-15 && cos(x) > 0.0 {
                return Some(x);
            }
        }
        if (sa > 0.0) == (sb > 0.0) {
            // Where the direction leaves along the domain's end, rounding can put the crossing a
            // hair beyond it.
            for (x, s) in [(a, sa), (b, sb)] {
                if (x == lo || x == hi) && s.abs() <= 1e-9 && cos(x) > 0.0 {
                    return Some(x);
                }
            }
            continue;
        }
        for _ in 0..64 {
            let m = 0.5 * (a + b);
            if (sine(m) > 0.0) == (sa > 0.0) {
                a = m;
            } else {
                b = m;
            }
        }
        return Some(0.5 * (a + b));
    }
    let (mut a, mut b) = (at(best.saturating_sub(1)), at((best + 1).min(SAMPLES)));
    // Golden-section search on the bracket round the best sample.
    let g = 0.5 * (5f64.sqrt() - 1.0);
    for _ in 0..60 {
        let (c, d) = (b - g * (b - a), a + g * (b - a));
        if cos(c) >= cos(d) {
            b = d;
        } else {
            a = c;
        }
    }
    Some(0.5 * (a + b))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        edit::{Affine, affine},
        math::{Frame, Transform},
        primitive::sphere,
    };

    // A direction turning once round on [-π, π] and one a hair past its end, as rounding leaves a
    // pole's line along a spline's seam: the end is the answer, not the flat peak's 1e-8 guess.
    #[test]
    fn a_direction_a_hair_past_the_domains_end_is_that_end() {
        use std::f64::consts::PI;
        let leaving = |x: f64| Some(Vec3::new(x.cos(), x.sin(), 0.0));
        for past in [1e-13, -1e-13, 0.0] {
            for (end, sign) in [(PI, 1.0), (-PI, -1.0)] {
                let a = end + sign * past;
                let found = best_of(&leaving, Vec3::new(a.cos(), a.sin(), 0.0), (-PI, PI)).unwrap();
                let off = (found - end).abs().min((found + end).abs());
                assert!(off <= 1e-10, "{past} past {end}: {found}");
            }
        }
    }

    // A ball stretched and sheared has a spline skin with a pole at each end of `v`. Each `u`'s
    // line leaves a pole along its own direction, and that direction gives the same `u` back
    // wherever the ball is placed: a coedge ending at the pole takes it, and the pole's own coedge
    // keeps the one found where the part was built. At the seam it is the domain's end exactly.
    #[test]
    fn a_direction_leaving_a_pole_names_its_parameter_to_a_hair_wherever_the_surface_is() {
        let map = Affine {
            m: [
                [0.1028, -0.9280, 0.5227],
                [-0.6650, 0.2041, 0.6977],
                [1.2920, 0.5654, 0.3662],
            ],
            t: Vec3::new(20.147, -35.624, 7.546),
        };
        let f = Frame::new(Vec3::new(-2.9, -1.3, -1.2), Vec3::Z, Vec3::X).unwrap();
        let ball = affine(&sphere(&f, 1.8, "sph").unwrap(), &map).unwrap();
        let s = &ball.face(ball.face_ids().next().unwrap()).surface;
        let Surface::Nurbs(n) = s else {
            panic!("a stretched ball's skin is a spline")
        };
        let ((u0, u1), (v0, v1)) = n.domain();
        let moves = [
            Vec3::ZERO,
            Vec3::new(100.0, -50.0, 300.0),
            Vec3::new(-731.0, 0.4, 0.0),
        ];
        let mut placed = Vec::new();
        for (k, moved) in moves.into_iter().enumerate() {
            for turn in 0..8 {
                let axis = Vec3::new(1.0, k as f64 - 1.0, 0.3 * turn as f64)
                    .unit()
                    .unwrap();
                let t = Transform::rotation(Vec3::ZERO, axis, 0.77 * turn as f64)
                    .then(&Transform::translation(moved));
                placed.push((k, t));
            }
        }
        for (k, t) in placed {
            let Surface::Nurbs(n) = s.transformed(&t) else {
                unreachable!()
            };
            for (pole, inward) in [(v0, 1.0), (v1, -1.0)] {
                assert!(
                    n.d1(0.3, pole).1.norm() <= tol::LINEAR,
                    "v {pole} is a pole"
                );
                for j in 0..=12 {
                    let off = if j == 0 || j == 12 { 0.0 } else { 0.01 };
                    let u = u0 + (u1 - u0) * j as f64 / 12.0 + off;
                    let away = (n.d1(u, pole).2 * inward).unit().unwrap();
                    let found = leaving_u(&n, pole, away).unwrap();
                    // The seam's two ends are one line.
                    let per = u1 - u0;
                    let off = (found - u) - ((found - u) / per).round() * per;
                    assert!(off.abs() <= 1e-10, "move {k}, pole {pole}: {found} for {u}");
                }
            }
        }
    }
}
