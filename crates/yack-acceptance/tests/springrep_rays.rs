// yack: rays against solids, against closed forms: where they cross, how often, and the normal there.
// Adapted from springrep@a03e8a1 tests/rays.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: springrep API replaced by the yack public API (PLAN §4.3); balls, boxes, cylinders and the
// torus are made with make::sphere / block / cylinder / torus on the same frames (springrep circles
// take a diameter); Rays::cast becomes Solid::raycast, which has no range, so every cast here is
// unbounded (the originals' ranges hold every crossing except where noted); the hit normal is read
// with Solid::normal_at at the hit point; the outline spline through 24 points is Loop::through and
// its curve is sampled with EdgeRef::points (no evaluation at a parameter: API gap noted there).

use std::f64::consts::{PI, TAU};

use yack::prelude::*;

fn ball(c: [f64; 3], r: f64) -> Solid {
    sphere(&Frame::at(c), r).unwrap()
}

fn cuboid(lo: [f64; 3], hi: [f64; 3]) -> Solid {
    block(&Frame::at(lo), [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]]).unwrap()
}

fn cast(b: &Solid, o: Point3, d: Vec3) -> Vec<f64> {
    b.raycast(&[Ray::new(o, d).unwrap()]).unwrap().remove(0)
}

fn assert_ts(got: &[f64], want: &[f64]) {
    assert_eq!(got.len(), want.len(), "{got:?} vs {want:?}");
    for (g, w) in got.iter().zip(want) {
        assert!((g - w).abs() <= 1e-9 * (1.0 + w.abs()), "{got:?} vs {want:?}");
    }
}

fn unit(v: Vec3) -> Vec3 {
    v * (1.0 / v.length())
}

#[test]
fn a_ray_beside_a_balls_seam_crosses_it_on_either_side() {
    // The seam is the meridian in y = 0, x > 0. A ray along z just either side of it is inside
    // the face on both sides: the seam bounds it both ways. A second ball, of radius 2 about
    // (-0.5, -1, 1.5), cuts the face up towards its pole, where a chord in the parameters strays
    // far across the surface, past the seam's distance from where the rays leave. Each ray passes
    // through the second ball inside the first, so it crosses the first at ±√(9 − x² − y²) only.
    let b = ball([0.0; 3], 3.0).union(&ball([-0.5, -1.0, 1.5], 2.0)).unwrap();
    let x: f64 = 1.0;
    for y in [-0.05, -0.02, 0.02, 0.05] {
        let z = (9.0 - x * x - y * y).sqrt();
        let hits = cast(&b, Point3::new(x, y, -10.0), Vec3::new(0.0, 0.0, 1.0));
        assert_ts(&hits, &[10.0 - z, 10.0 + z]);
    }
}

#[test]
fn a_ray_through_a_ball_crosses_twice_touching_once_and_missing_never() {
    // A ray along x passing h from the centre crosses at ±√(r² − h²) about its nearest approach.
    let (c, r) = (Point3::new(1.0, 2.0, 3.0), 3.0);
    let b = ball(c.to_array(), r);
    let x = Vec3::new(1.0, 0.0, 0.0);
    let o = c + Vec3::new(-10.0, 1.0, 0.0);
    let hits = cast(&b, o, x);
    let half = (r * r - 1.0f64).sqrt();
    assert_ts(&hits, &[10.0 - half, 10.0 + half]);
    // The normal points out of the ball: from the centre to the point.
    let points: Vec<Point3> = hits.iter().map(|&t| o + x * t).collect();
    for (p, n) in points.iter().zip(b.normal_at(&points)) {
        let n = n.unwrap();
        assert!(((*p - c) * (1.0 / r) - n).length() < 1e-9, "{p:?}: {n:?}");
    }
    // Grazing the ball at h = r touches it once; past it, nothing.
    assert_ts(&cast(&b, c + Vec3::new(-10.0, r, 0.0), x), &[10.0]);
    assert_ts(&cast(&b, c + Vec3::new(-10.0, 3.5, 0.0), x), &[]);
    // API GAP: Solid::raycast takes no range, so "a range (0, 10) that stops inside keeps the entry
    // only" (want [10 − half]) is dropped.
}

/// A cylinder of radius 50 km and height 40 km: rays from points inside it leave through its side
/// where the circle says, the far one along the ray's line in its plane. Each direction is
/// solved for exactly: the ray `p + t·d` meets `x² + y² = r²` at the positive root of a quadratic.
#[test]
fn rays_from_inside_a_cylinder_kilometres_across_leave_through_its_side() {
    let (r, h) = (5e4, 4e4);
    let b = cylinder(&Frame::WORLD, r, h).unwrap();
    // Directions that leave through the side before the caps from these points.
    let d = unit(Vec3::new(0.6213, -0.2841, 0.07302));
    for p in [
        Point3::new(-24471.3, -24.0, 9520.7),
        Point3::new(21381.1, 5364.3, 3236.0),
        Point3::new(-40844.9, 3285.6, 18088.7),
    ] {
        let (a, bq, c) = (d.x * d.x + d.y * d.y, 2.0 * (p.x * d.x + p.y * d.y), p.x * p.x + p.y * p.y - r * r);
        let t = (-bq + (bq * bq - 4.0 * a * c).sqrt()) / (2.0 * a);
        assert!((p + d * t).z < h, "the ray leaves through the side");
        // The original's range (0, 1e6) holds every crossing; raycast is unbounded.
        let hits = cast(&b, p, d);
        assert_ts(&hits, &[t]);
    }
}

/// A ray that clips a cylinder's side in a chord shallower than the caster's mesh resolves: it
/// crosses twice, in and out, at the two roots of the quadratic in the cylinder's plane. Chords of
/// 3 µm down to 3 nm deep, on a cylinder of 2 mm radius.
#[test]
fn a_ray_clipping_a_cylinder_shallower_than_the_mesh_crosses_it_twice() {
    let (r, h) = (2.0, 10.0);
    let b = cylinder(&Frame::WORLD, r, h).unwrap();
    let d = unit(Vec3::new(0.3713, 0.5347, 0.07591));
    let across = unit(Vec3::new(d.y, -d.x, 0.0));
    let steep = unit(Vec3::new(0.3713, 0.5347, 0.7591));
    let found = (Point3::new(0.6263603775040814, -2.600153099460448, -1.2639721248298266), steep);
    let shallow = [3e-3, 3e-4, 3e-5, 3e-6].map(|depth| {
        // A start whose line in the plane passes `depth` inside the circle.
        (Point3::ORIGIN + across * (r - depth) - d * 4.0 + Vec3::new(0.0, 0.0, 5.0), d)
    });
    for (p, d) in shallow.into_iter().chain([found]) {
        let (a, bq, c) = (d.x * d.x + d.y * d.y, p.x * d.x + p.y * d.y, p.x * p.x + p.y * p.y - r * r);
        let root = (bq * bq - a * c).sqrt();
        // The original's range (0, 100) holds every crossing; raycast is unbounded.
        let hits = cast(&b, p, d);
        assert_ts(&hits, &[(-bq - root) / a, (-bq + root) / a]);
        let at = [p + d * hits[0], p + d * hits[1]];
        let n = b.normal_at(&at);
        assert!(n[0].unwrap().dot(d) < 0.0 && n[1].unwrap().dot(d) > 0.0);
    }
}

/// The lens two balls a few micrometres across share: rays from a point inside it leave where the
/// nearer ball's sphere says, `−b + √(b² − c)` along the unit direction for each sphere, the nearer
/// of the two. A part far smaller than the caster's own mesh deflection is still found.
#[test]
fn rays_from_inside_a_lens_micrometres_across_leave_where_its_spheres_say() {
    let (ca, ra) = (Point3::ORIGIN, 0.004054250852155359);
    let (cb, rb) = (Point3::new(0.00338806114604008, 0.0, 0.0), 0.0026689525243786373);
    let lens = ball(ca.to_array(), ra).intersection(&ball(cb.to_array(), rb)).unwrap();
    let p = Point3::new(0.001017, -0.000028, -0.000248);
    for d in [
        Vec3::new(0.3713, 0.5347, 0.7591),
        Vec3::new(-0.6213, 0.2841, 0.7302),
        Vec3::new(0.4472, -0.8018, 0.3961),
    ] {
        let d = unit(d);
        let exit = |c: Point3, r: f64| {
            let (b, cc) = ((p - c).dot(d), (p - c).dot(p - c) - r * r);
            -b + (b * b - cc).sqrt()
        };
        assert_ts(&cast(&lens, p, d), &[exit(ca, ra).min(exit(cb, rb))]);
    }
}

#[test]
fn a_ray_through_a_box_enters_and_leaves_where_the_slabs_say() {
    // x 0..10, y 0..6, z 0..3, crossed on a skew line: each axis's slab gives the t interval it
    // lies in, and the entry and exit are the latest start and the earliest end.
    let b = cuboid([0.0; 3], [10.0, 6.0, 3.0]);
    let o = Point3::new(-5.0, 2.0, 1.0);
    let d = unit(Vec3::new(1.0, 0.1, 0.05));
    let slab = |o: f64, d: f64, lo: f64, hi: f64| {
        let (a, z) = ((lo - o) / d, (hi - o) / d);
        (a.min(z), a.max(z))
    };
    let s = [slab(o.x, d.x, 0.0, 10.0), slab(o.y, d.y, 0.0, 6.0), slab(o.z, d.z, 0.0, 3.0)];
    let enter = s.iter().map(|x| x.0).fold(f64::MIN, f64::max);
    let leave = s.iter().map(|x| x.1).fold(f64::MAX, f64::min);
    assert_ts(&cast(&b, o, d), &[enter, leave]);
}

#[test]
fn a_ray_through_an_edge_crosses_there_once() {
    // From (−1, −1) along the diagonal at mid-height: in at the edge x = y = 0 after √2, out
    // through the face y = 6 at (6, 6) after 7√2.
    let b = cuboid([0.0; 3], [10.0, 6.0, 3.0]);
    let d = Vec3::new(1.0, 1.0, 0.0);
    let hits = cast(&b, Point3::new(-1.0, -1.0, 1.5), d);
    assert_ts(&hits, &[2.0f64.sqrt(), 7.0 * 2.0f64.sqrt()]);
}

#[test]
fn a_ray_along_a_torus_diameter_crosses_four_times_and_down_its_axis_none() {
    // Tube radius 2 at R = 10 round z: along x through the centre at x = ±8 and ±12.
    let t = torus(&Frame::WORLD, 10.0, 4.0 / 2.0).unwrap();
    let o = Point3::new(-20.0, 0.0, 0.0);
    assert_ts(&cast(&t, o, Vec3::new(1.0, 0.0, 0.0)), &[8.0, 12.0, 28.0, 32.0]);
    assert_ts(&cast(&t, Point3::new(0.0, 0.0, -20.0), Vec3::new(0.0, 0.0, 1.0)), &[]);
}

#[test]
fn a_ray_across_a_spline_extrusion_crosses_where_the_outline_does() {
    // A disc whose outline is a periodic spline, 10 high, crossed at mid-height along y = 9: the
    // crossings are where the outline passes y = 9, found by sampling its curve and bisecting
    // each change of side, which shares nothing with the ray's search.
    let pts: Vec<[f64; 2]> = (0..24)
        .map(|k| {
            let a = TAU * k as f64 / 24.0;
            [30.0 * a.cos(), 20.0 * a.sin() + 3.0 * (3.0 * a).sin()]
        })
        .collect();
    let p = Profile::new(Loop::through(&pts).unwrap(), vec![]).unwrap();
    let disc = extrude(&Frame::WORLD, &p, 10.0).unwrap();
    let outline =
        disc.edges_where(|e| e.midpoint().z.abs() < 1e-9 && !matches!(e.kind(), CurveKind::Line { .. }))[0];
    let side = |q: Point3| q.y - 9.0;
    let o = Point3::new(-40.0, 9.0, 5.0);
    // API GAP: EdgeRef cannot be evaluated at a parameter, so the original's 20 000 intervals each
    // bisected 200 times are replaced by 2 000 000 intervals of EdgeRef::points (evenly in the
    // parameter, ends included), the crossing interpolated linearly within its interval.
    let n = 2_000_000;
    let samples = disc.edge(outline).unwrap().points(n + 1);
    let mut want = Vec::new();
    for w in samples.windows(2) {
        let (lo, hi) = (w[0], w[1]);
        if side(lo).signum() == side(hi).signum() {
            continue;
        }
        let f = side(lo) / (side(lo) - side(hi));
        want.push(lo.x + (hi.x - lo.x) * f - o.x);
    }
    want.sort_by(f64::total_cmp);
    assert!(want.len() >= 2, "{want:?}");
    assert_ts(&cast(&disc, o, Vec3::new(1.0, 0.0, 0.0)), &want);
}

#[test]
fn a_ray_just_inside_a_curved_rim_crosses_both_ends() {
    // 0.2 µm inside a 3 mm disc's rim, between the outline's samples, where its chords cut 0.9 µm
    // inside the arc: the ends' polygons would leave the point out of both faces.
    let disc = cylinder(&Frame::WORLD, 6.0 / 2.0, 4.0).unwrap();
    let a = PI / 128.0;
    let r = 3.0 - 2e-4;
    let o = Point3::new(r * a.cos(), r * a.sin(), -5.0);
    let hits = cast(&disc, o, Vec3::new(0.0, 0.0, 1.0));
    assert_ts(&hits, &[5.0, 9.0]);
}
