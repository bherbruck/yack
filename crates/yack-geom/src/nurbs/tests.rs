// yack: tests of the rational B-spline curves and surfaces.
// Adapted from springrep@a03e8a1 src/geom/nurbs.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/geom/nurbs.rs to yack-geom (M3 W3); split out of
// nurbs.rs into its own file (M3 hardening); paths only.
//! Tests of the rational B-spline curves and surfaces.

use super::*;
use crate::math::{Point3, Vec2, Vec3};
use std::f64::consts::{FRAC_1_SQRT_2, TAU};

/// The unit circle as a rational quadratic in four quarter arcs.
fn circle() -> NurbsCurve {
    let w = FRAC_1_SQRT_2;
    let pts = [
        (1.0, 0.0),
        (1.0, 1.0),
        (0.0, 1.0),
        (-1.0, 1.0),
        (-1.0, 0.0),
        (-1.0, -1.0),
        (0.0, -1.0),
        (1.0, -1.0),
        (1.0, 0.0),
    ];
    let ctrl = pts.iter().map(|&(x, y)| Vec3::new(x, y, 0.0)).collect();
    let weights = (0..9).map(|i| if i % 2 == 0 { 1.0 } else { w }).collect();
    let knots = vec![0.0, 0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 3.0, 3.0, 4.0, 4.0, 4.0];
    NurbsCurve::new(2, knots, ctrl, Some(weights)).unwrap()
}

#[test]
fn rational_circle_is_exact_and_periodic() {
    let c = circle();
    assert_eq!(c.period(), Some(4.0));
    for i in 0..=40 {
        let t = -2.0 + 8.0 * i as f64 / 40.0;
        let (p, d1, d2) = c.d2(t);
        assert!((p.norm() - 1.0).abs() < 1e-15, "{t}");
        assert!(p.dot(d1).abs() < 1e-14);
        let h = 1e-5;
        let fd = (c.eval(t + h) - c.eval(t - h)) / (2.0 * h);
        let fd2 = (c.eval(t + h) - p * 2.0 + c.eval(t - h)) / (h * h);
        if (t - t.round()).abs() > 2.0 * h {
            assert!(d1.dist(fd) < 1e-8, "{t}");
            assert!(d2.dist(fd2) < 1e-4, "{t}");
        }
    }
}

#[test]
fn surface_derivatives_and_projection() {
    // A quarter of a cylinder of radius 1, height 2, as a rational quadratic by linear patch.
    let w = FRAC_1_SQRT_2;
    let arc = [(1.0, 0.0, 1.0), (1.0, 1.0, w), (0.0, 1.0, 1.0)];
    let mut ctrl = Vec::new();
    let mut weights = Vec::new();
    for &(x, y, wt) in &arc {
        for z in [0.0, 2.0] {
            ctrl.push(Vec3::new(x, y, z));
            weights.push(wt);
        }
    }
    let s = NurbsSurface::new(
        (2, 1),
        (vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], vec![0.0, 0.0, 1.0, 1.0]),
        (3, 2),
        ctrl,
        Some(weights),
    )
    .unwrap();
    assert_eq!(s.u_period(), None);
    for &(u, v) in &[(0.3, 0.4), (0.9, 0.1), (0.5, 0.5)] {
        let (p, su, sv) = s.d1(u, v);
        assert!((p.x.hypot(p.y) - 1.0).abs() < 1e-15);
        let h = 1e-6;
        let fu = (s.eval(u + h, v) - s.eval(u - h, v)) / (2.0 * h);
        let fv = (s.eval(u, v + h) - s.eval(u, v - h)) / (2.0 * h);
        assert!(su.dist(fu) < 1e-8 && sv.dist(fv) < 1e-8);
        let uv = s.project(p + su.cross(sv).unit().unwrap() * 0.5);
        assert!(
            (uv.x - u).abs() < 1e-12 && (uv.y - v).abs() < 1e-12,
            "{uv:?}"
        );
        let near = s.project_near(p, Vec2::new(0.5, 0.5));
        assert!(s.eval(near.x, near.y).dist(p) < 1e-12);
    }
    let angle = TAU / 4.0;
    assert!(
        s.eval(1.0, 0.0)
            .dist(Vec3::new(angle.cos(), angle.sin(), 0.0))
            < 1e-15
    );
}

/// A rational bicubic with a twist and uneven weights, two spans along `u`.
fn warped() -> NurbsSurface {
    let (nu, nv) = (5, 4);
    let mut ctrl = Vec::new();
    let mut weights = Vec::new();
    for i in 0..nu {
        for j in 0..nv {
            let (x, y) = (i as f64, j as f64);
            let a = 0.3 * x * y;
            ctrl.push(Vec3::new(
                x * a.cos(),
                y + x * a.sin(),
                (x - 2.0) * (y - 1.5) * 0.4,
            ));
            weights.push(1.0 + 0.1 * ((i + 2 * j) % 3) as f64);
        }
    }
    NurbsSurface::new(
        (3, 3),
        (
            vec![0.0, 0.0, 0.0, 0.0, 0.4, 1.0, 1.0, 1.0, 1.0],
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        ),
        (nu, nv),
        ctrl,
        Some(weights),
    )
    .unwrap()
}

/// The least distance from `p` to a grid of the patch's points, its boundary among them: no
/// point of the patch is much nearer than this, and its nearest point is no farther.
fn nearest_sampled(s: &NurbsSurface, p: Point3, n: usize) -> f64 {
    let mut best = f64::INFINITY;
    for i in 0..=n {
        for j in 0..=n {
            let q = s.eval(i as f64 / n as f64, j as f64 / n as f64);
            best = best.min(q.dist(p));
        }
    }
    best
}

#[test]
fn a_point_beyond_an_edge_projects_onto_the_nearest_point_of_that_edge() {
    let s = warped();
    // Points off each of the four edges, a way along and a way out, and off a corner.
    let mut tried = 0;
    for k in 1..12 {
        let t = k as f64 / 12.0;
        for (on, inside) in [
            (Vec2::new(t, 1.0), Vec2::new(t, 0.9)),
            (Vec2::new(t, 0.0), Vec2::new(t, 0.1)),
            (Vec2::new(1.0, t), Vec2::new(0.9, t)),
            (Vec2::new(0.0, t), Vec2::new(0.1, t)),
        ] {
            let (edge, back) = (s.eval(on.x, on.y), s.eval(inside.x, inside.y));
            let out = (edge - back).unit().unwrap();
            let [_, su, sv, ..] = s.d2(on.x, on.y);
            let lift = su.cross(sv).unit().unwrap() * 0.05;
            for reach in [0.02, 0.2, 0.6] {
                let p = edge + out * reach + lift;
                let uv = s.project(p);
                let got = s.eval(uv.x, uv.y).dist(p);
                let sampled = nearest_sampled(&s, p, 600);
                assert!(
                    got <= sampled + 1e-9,
                    "off {on:?} by {reach}: {got} against {sampled} at {uv:?}"
                );
                tried += 1;
            }
        }
    }
    assert_eq!(tried, 132);
}

/// The point of a Bézier patch at `(s, t)` in its own unit square, by de Casteljau.
fn bezier_point(b: &Bezier, s: f64, t: f64) -> Point3 {
    let fold = |mut level: Vec<[f64; 4]>, x: f64| {
        while level.len() > 1 {
            level = level
                .windows(2)
                .map(|w| std::array::from_fn(|c| (1.0 - x) * w[0][c] + x * w[1][c]))
                .collect();
        }
        level[0]
    };
    let rows: Vec<[f64; 4]> = (0..b.rows)
        .map(|i| fold(b.net[i * b.cols..(i + 1) * b.cols].to_vec(), t))
        .collect();
    let q = fold(rows, s);
    Vec3::new(q[0] / q[3], q[1] / q[3], q[2] / q[3])
}

#[test]
fn bezier_patches_are_the_surface_and_their_halves_are_too() {
    for s in uneven().iter().chain([&warped()]) {
        let mut pieces: Vec<Bezier> = s.beziers().to_vec();
        assert_eq!(pieces.len(), s.patches().len());
        // Each patch, and the quarters two cuts leave of the first few.
        let quarters: Vec<Bezier> = pieces
            .iter()
            .take(6)
            .flat_map(|b| b.halves(true))
            .flat_map(|b| b.halves(false))
            .collect();
        pieces.extend(quarters);
        for b in &pieces {
            let bb = b.bbox();
            for (x, y) in [(0.0, 0.0), (1.0, 1.0), (0.3, 0.8), (0.5, 0.5), (0.9, 0.1)] {
                let (u, v) = (
                    b.lo.x + (b.hi.x - b.lo.x) * x,
                    b.lo.y + (b.hi.y - b.lo.y) * y,
                );
                // A hair inside the far sides, where the patch's own spans end.
                let on = s.eval(u.min(b.hi.x - 1e-12), v.min(b.hi.y - 1e-12));
                let got = bezier_point(b, x, y);
                assert!(got.dist(on) < 1e-9, "{got:?} against {on:?}");
                let inside = (0..3).all(|k| {
                    on.to_array()[k] >= bb.min.to_array()[k] - 1e-9
                        && on.to_array()[k] <= bb.max.to_array()[k] + 1e-9
                });
                assert!(inside, "{on:?} outside {bb:?}");
            }
        }
    }
}

#[test]
fn an_iso_curve_is_the_surface_along_one_parameter() {
    let mut surfaces = vec![warped()];
    surfaces.extend(uneven());
    for s in surfaces {
        let ((u0, u1), (v0, v1)) = s.domain();
        let size = s.eval(u0, v0).norm() + 1.0;
        for along_v in [true, false] {
            for a in [0.0, 0.3, 0.5, 0.75, 1.0] {
                let at = if along_v {
                    u0 + (u1 - u0) * a
                } else {
                    v0 + (v1 - v0) * a
                };
                let c = s.iso_curve(along_v, at).unwrap();
                let (t0, t1) = c.domain();
                assert_eq!((t0, t1), if along_v { (v0, v1) } else { (u0, u1) });
                for b in [0.0, 0.2, 0.45, 0.5, 0.9, 1.0] {
                    let t = t0 + (t1 - t0) * b;
                    let want = if along_v {
                        s.eval(at, t)
                    } else {
                        s.eval(t, at)
                    };
                    let off = c.eval(t).dist(want);
                    assert!(off < 1e-12 * size, "{off} at {at}, {t}, along v {along_v}");
                }
            }
        }
    }
}

#[test]
fn iso_lines_agree_with_the_surface() {
    let s = warped();
    for &(at, t) in &[(0.2, 0.3), (0.55, 0.8), (0.4, 0.4), (1.0, 0.0)] {
        for along_v in [true, false] {
            let (u, v) = if along_v { (at, t) } else { (t, at) };
            let (p, su, sv) = s.iso(along_v, at).d1(t);
            let (q, qu, qv) = s.d1(u, v);
            assert!(p.dist(q) < 1e-14 && su.dist(qu) < 1e-13 && sv.dist(qv) < 1e-13);
        }
    }
    // The span polynomials evaluate up to degree 6, across a double knot and on both sides of
    // each span's middle, where the series switches ends; past it the basis recurrence does.
    for s in uneven() {
        let ((u0, u1), (v0, v1)) = s.domain();
        let one = s.iso(true, u0).d1(v0);
        let size = one.0.norm() + one.1.norm() + one.2.norm();
        for (a, b) in [
            (0.0, 0.0),
            (0.15, 0.9),
            (0.55, 0.45),
            (0.5, 0.5),
            (1.0, 1.0),
        ] {
            for u in [u0 + (u1 - u0) * a, 1.1] {
                let v = v0 + (v1 - v0) * b;
                let (q, qu, qv) = s.d1(u, v);
                for (p, su, sv) in [s.iso(true, u).d1(v), s.iso(false, v).d1(u)] {
                    let off = p.dist(q).max(su.dist(qu)).max(sv.dist(qv));
                    assert!(off < 1e-12 * size, "{off} at ({u}, {v})");
                }
            }
        }
    }
}

/// A degree-(3, 6) polynomial surface with a double interior knot along `u`, a degree-(6, 6)
/// rational one, the highest the span polynomials take, and a degree-12 rational one: spans of
/// unequal width, a knot where the surface is only C¹, a high degree.
fn uneven() -> [NurbsSurface; 3] {
    let net = |nu: usize, nv: usize, rational: bool| {
        let mut ctrl = Vec::new();
        let mut weights = Vec::new();
        for i in 0..nu {
            for j in 0..nv {
                let (x, y) = (i as f64, j as f64);
                ctrl.push(Vec3::new(x + 0.2 * y, y - 0.1 * x * x, (x * 0.7).sin() * y));
                weights.push(0.6 + 0.15 * ((3 * i + j) % 5) as f64);
            }
        }
        (ctrl, rational.then_some(weights))
    };
    let (c1, _) = net(7, 8, false);
    let a = NurbsSurface::new(
        (3, 6),
        (
            vec![0.0, 0.0, 0.0, 0.0, 0.3, 1.1, 1.1, 2.0, 2.0, 2.0, 2.0],
            vec![
                -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, 0.5, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0,
                2.0,
            ],
        ),
        (7, 8),
        c1,
        None,
    )
    .unwrap();
    let (c3, w3) = net(9, 8, true);
    let c = NurbsSurface::new(
        (6, 6),
        (
            vec![
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.7, 1.9, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0,
            ],
            vec![1.0; 7]
                .into_iter()
                .chain([2.5])
                .chain(vec![4.0; 7])
                .collect(),
        ),
        (9, 8),
        c3,
        w3,
    )
    .unwrap();
    let (c2, w2) = net(13, 4, true);
    let mut ku = vec![0.0; 13];
    ku.extend(vec![1.0; 13]);
    let b = NurbsSurface::new(
        (12, 2),
        (ku, vec![0.0, 0.0, 0.0, 0.25, 1.0, 1.0, 1.0]),
        (13, 4),
        c2,
        w2,
    )
    .unwrap();
    [a, c, b]
}

#[test]
fn span_polynomials_agree_with_the_basis_recurrence() {
    // Each span's polynomial must give what the Cox–de Boor recurrence does: the point and
    // every derivative up to the second, at span ends, inside spans and on a double knot.
    let mut surfaces = uneven().to_vec();
    surfaces.push(warped());
    // Interior knots of full multiplicity: C⁰ joints, as a corner patch of three arcs has.
    let mut ctrl = Vec::new();
    let mut weights = Vec::new();
    for i in 0..7 {
        for j in 0..3 {
            let a = 0.9 * i as f64;
            ctrl.push(Vec3::new(
                a.cos() * (2.0 - j as f64),
                a.sin() * (2.0 - j as f64),
                j as f64,
            ));
            weights.push(if i % 2 == 1 { FRAC_1_SQRT_2 } else { 1.0 });
        }
    }
    surfaces.push(
        NurbsSurface::new(
            (2, 2),
            (
                vec![0.0, 0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 3.0, 3.0, 3.0],
                vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            ),
            (7, 3),
            ctrl,
            Some(weights),
        )
        .unwrap(),
    );
    for s in &surfaces {
        let ((u0, u1), (v0, v1)) = s.domain();
        let mut knots: Vec<f64> = s.bu.knots.clone();
        knots.extend((0..=37).map(|k| u0 + (u1 - u0) * k as f64 / 37.0));
        let mut vk: Vec<f64> = s.bv.knots.clone();
        vk.extend((0..=29).map(|k| v0 + (v1 - v0) * k as f64 / 29.0));
        for &u in knots.iter().filter(|&&u| u >= u0 && u <= u1) {
            for &v in vk.iter().filter(|&&v| v >= v0 && v <= v1) {
                let got = s.d2(u, v);
                let want = s.d2_n::<M>(u, v);
                // Rounding in the rational quotient grows with each derivative taken.
                let tol = [1e-12, 1e-11, 1e-11, 1e-10, 1e-10, 1e-10];
                for (k, (g, w)) in got.iter().zip(&want).enumerate() {
                    let size = 1.0 + w.norm();
                    assert!(
                        g.dist(*w) <= tol[k] * size,
                        "order {k} at ({u}, {v}): {g:?} against {w:?}"
                    );
                }
                let (p, su, sv) = s.d1(u, v);
                let (q, qu, qv) = s.d1_n::<M>(u, v);
                assert!(p.dist(q) <= 1e-12 * (1.0 + q.norm()));
                assert!(su.dist(qu) <= 1e-11 * (1.0 + qu.norm()));
                assert!(sv.dist(qv) <= 1e-11 * (1.0 + qv.norm()));
                assert!(s.eval(u, v).dist(q) <= 1e-12 * (1.0 + q.norm()));
            }
        }
    }
}

#[test]
fn second_derivatives_match_differences() {
    let s = warped();
    let h = 1e-5;
    for &(u, v) in &[(0.2, 0.3), (0.55, 0.8), (0.9, 0.1)] {
        let [_, su, sv, suu, suv, svv] = s.d2(u, v);
        let (_, su0, sv0) = s.d1(u, v);
        assert!(su.dist(su0) < 1e-12 && sv.dist(sv0) < 1e-12);
        let du = |u: f64, v: f64| s.d1(u, v).1;
        let dv = |u: f64, v: f64| s.d1(u, v).2;
        assert!(suu.dist((du(u + h, v) - du(u - h, v)) / (2.0 * h)) < 1e-6);
        assert!(suv.dist((du(u, v + h) - du(u, v - h)) / (2.0 * h)) < 1e-6);
        assert!(svv.dist((dv(u, v + h) - dv(u, v - h)) / (2.0 * h)) < 1e-6);
    }
}

#[test]
fn patch_boxes_hold_the_surface_over_their_parameters() {
    let s = warped();
    let patches = s.patches();
    assert!(patches.len() > 2);
    for i in 0..=60 {
        for j in 0..=60 {
            let uv = Vec2::new(i as f64 / 60.0, j as f64 / 60.0);
            let p = s.eval(uv.x, uv.y);
            let holding = patches.iter().filter(|q| {
                uv.x >= q.lo.x && uv.x <= q.hi.x && uv.y >= q.lo.y && uv.y <= q.hi.y
            });
            let mut any = false;
            for q in holding {
                any = true;
                let pad = 1e-12;
                assert!(
                    (0..3).all(|k| p[k] >= q.bbox.min[k] - pad && p[k] <= q.bbox.max[k] + pad),
                    "{uv:?}"
                );
            }
            assert!(any, "{uv:?}");
        }
    }
}

#[test]
fn projection_from_off_the_surface_lands_on_the_nearest_point() {
    let s = warped();
    for &(u, v, off) in &[(0.3, 0.6, 0.05), (0.7, 0.2, -0.08), (0.45, 0.45, 0.02)] {
        let (p, su, sv) = s.d1(u, v);
        let q = p + su.cross(sv).unit().unwrap() * off;
        let uv = s.project(q);
        assert!(
            (uv.x - u).abs() < 1e-10 && (uv.y - v).abs() < 1e-10,
            "{uv:?}"
        );
        let near = s.project_near(q, Vec2::new(u + 0.02, v - 0.02));
        assert!(near.dist(uv) < 1e-10, "{near:?}");
    }
}

/// The octant of the sphere of radius `r` about the origin, as the rational biquadratic of
/// Piegl and Tiller: `u` round the equator from +x to +y, `v` up to the pole at +z, `v = 1`.
pub(crate) fn octant(r: f64) -> crate::Surface {
    let w = FRAC_1_SQRT_2;
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
    crate::Surface::Nurbs(Box::new(n))
}

#[test]
fn projection_beside_a_pole_lands_on_the_nearest_point() {
    // Every seed of the pole's row is the pole: Newton from there never moves, so the point
    // must be found from the seeds beside it. A point at polar angle θ and azimuth φ, lifted
    // off the sphere by `off`, has its nearest point on the sphere at the same angles.
    let r = 2.0;
    let s = octant(r);
    for theta in [3e-8f64, 1e-4, 1e-3, 1e-2, 5e-2] {
        for phi in [0.2f64, 0.7, 1.3] {
            for off in [0.0, 1e-3, -1e-3] {
                let dir = Vec3::new(
                    theta.sin() * phi.cos(),
                    theta.sin() * phi.sin(),
                    theta.cos(),
                );
                let uv = s.project(dir * (r + off));
                let got = s.eval(uv.x, uv.y);
                assert!(got.dist(dir * r) < 1e-11, "θ {theta} φ {phi}: {got:?}");
                // From a hint on the pole itself, where every `u` is the same point.
                let uv = s.project_near(dir * (r + off), Vec2::new(0.5, 1.0));
                let got = s.eval(uv.x, uv.y);
                assert!(
                    got.dist(dir * r) < 1e-11,
                    "near: θ {theta} φ {phi}: {got:?}"
                );
                // From a hint just off the pole on the far boundary line, `u = 1` (φ = π/2):
                // `su` there is under the tolerance, yet the hint's `u` is not the same point.
                let uv = s.project_near(dir * (r + off), Vec2::new(1.0, 1.0 - 1e-8));
                let got = s.eval(uv.x, uv.y);
                assert!(
                    got.dist(dir * r) < 1e-11,
                    "beside: θ {theta} φ {phi}: {got:?}"
                );
            }
        }
    }
}
