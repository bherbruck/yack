// yack: the rolling ball held to the closed forms of a cylinder cut by a plane at 45° and grown.
//! A cylinder of radius 3 about z, from z = −10 up to the plane z = x, grown by `d`: every face moved
//! out by `d` and every edge rounded by a ball of radius `d`. The ball over the top's elliptic edge
//! rolls with its centre on the ellipse it grew from (x² + y² = 9 on z = x), and the grown solid is
//! K's parallel body, with Steiner's volume and area. Past `d` ≈ 1.4 the moved edge's square planes
//! cross within the ball's reach, so these runs exercise the spine rule.

use crate::{
    boolean, check, measure, sweep,
    geom::Surface,
    math::{Frame, Point3, Vec2, Vec3},
    profile::Profile,
    topo::{Brep, EdgeId},
};
use std::collections::BTreeSet;
use std::f64::consts::{FRAC_1_SQRT_2, PI, SQRT_2};

/// The slant-cut cylinder's faces moved out by `d`: radius 3 + d, from z = −10 − d up to the
/// plane z = x + d√2.
fn moved(d: f64) -> Brep {
    let r = 3.0 + d;
    let bottom = Frame::new(Vec3::new(0.0, 0.0, -10.0 - d), Vec3::Z, Vec3::X).unwrap();
    let rod = sweep::extrude(&Profile::circle(Vec2::ZERO, 2.0 * r), &bottom, 10.0 + 3.0 * r + 3.0 * d, "rod").unwrap();
    let up = Vec3::new(-FRAC_1_SQRT_2, 0.0, FRAC_1_SQRT_2);
    let above = Frame::new(Vec3::new(0.0, 0.0, d * SQRT_2), up, Vec3::Y).unwrap();
    let slab = sweep::extrude(&Profile::rect(Vec2::ZERO, 8.0 * r, 8.0 * r), &above, 8.0 * r, "slab").unwrap();
    boolean::difference(&rod, &slab).unwrap()
}

/// Every edge between two faces (a seam is between one face and itself).
fn creases(b: &Brep) -> Vec<EdgeId> {
    b.edge_ids()
        .filter(|&e| {
            let faces: BTreeSet<_> =
                b.coedge_ids().filter(|&c| b.coedge(c).edge == e).map(|c| b.coedge_face(c)).collect();
            faces.len() == 2
        })
        .collect()
}

/// The distance from `p` to the ellipse the top's ball rolls along: x² + y² = 9 on z = x.
fn from_spine(p: Point3) -> f64 {
    let at = |t: f64| p.dist(Point3::new(3.0 * t.cos(), 3.0 * t.sin(), 3.0 * t.cos()));
    let n = 720;
    let (mut lo, mut hi) = (0..n)
        .map(|i| 2.0 * PI * i as f64 / n as f64)
        .min_by(|a, b| at(*a).total_cmp(&at(*b)))
        .map(|t| (t - 2.0 * PI / n as f64, t + 2.0 * PI / n as f64))
        .unwrap();
    for _ in 0..100 {
        let (a, b) = (lo + (hi - lo) / 3.0, hi - (hi - lo) / 3.0);
        if at(a) < at(b) { hi = b } else { lo = a }
    }
    at(0.5 * (lo + hi))
}

/// J, the top edge's turning angle integrated along it: at the ellipse's point
/// P(φ) = (3 cos φ, 3 sin φ, 3 cos φ) the angle between the top's outward normal (−1, 0, 1)/√2 and
/// the side's (cos φ, sin φ, 0), times the length element |P′(φ)|. The integrand is smooth and
/// periodic, so the trapezoid rule converges geometrically.
fn turning() -> f64 {
    let n = 4096;
    let h = 2.0 * PI / n as f64;
    let top = Vec3::new(-FRAC_1_SQRT_2, 0.0, FRAC_1_SQRT_2);
    (0..n)
        .map(|i| {
            let t = h * i as f64;
            let side = Vec3::new(t.cos(), t.sin(), 0.0);
            let along = Vec3::new(-3.0 * t.sin(), 3.0 * t.cos(), -3.0 * t.sin());
            top.dot(side).acos() * along.norm() * h
        })
        .sum()
}

#[test]
fn the_ball_grown_round_a_slant_cut_cylinder_rolls_on_the_ellipse_it_grew_from() {
    let j = turning();
    let (v0, s0, m) = (90.0 * PI, 69.0 * PI + 9.0 * PI * SQRT_2, 10.0 * PI + 1.5 * PI * PI + 0.5 * j);
    for d in [1.5, 2.0, 2.5, 4.0] {
        let body = moved(d);
        let grown = crate::fillet_edges_mitred(&body, &creases(&body), d, "grown").unwrap();
        assert!(check::check(&grown).is_empty(), "d {d}: {:?}", check::check(&grown));
        let (area, volume) = measure::area_and_volume(&grown);
        let want_v = v0 + s0 * d + m * d * d + 4.0 * PI * d * d * d / 3.0;
        let want_a = s0 + 2.0 * m * d + 4.0 * PI * d * d;
        assert!((volume - want_v).abs() < 1e-6 * want_v, // tol: the fit's tenth of the tolerance, as a share
             "d {d}: volume {volume}, Steiner {want_v}");
        assert!((area - want_a).abs() < 1e-6 * want_a, // tol: the fit's tenth of the tolerance, as a share
             "d {d}: area {area}, Steiner {want_a}");
        // the canal face: every point at d from the ellipse (the bottom's torus lies below z = −10)
        let mut seen = 0;
        for f in grown.face_ids() {
            let Surface::Nurbs(s) = &grown.face(f).surface else { continue };
            let ((u0, u1), (w0, w1)) = s.domain();
            for i in 0..=8 {
                for k in 0..=4 {
                    let p = s.eval(u0 + (u1 - u0) * i as f64 / 8.0, w0 + (w1 - w0) * k as f64 / 4.0);
                    if p.z > -9.9 {
                        let off = from_spine(p) - d;
                        assert!(off.abs() < 1e-6, // tol: the fit's tenth of the tolerance and then some
                             "d {d}: a canal point {p:?} is {off} off the ball");
                        seen += 1;
                    }
                }
            }
        }
        assert!(seen > 0, "d {d}: no canal face");
    }
}
