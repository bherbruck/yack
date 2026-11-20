// yack: slabs (a block under a B-spline height field) against closed-form volume and area.
// Adapted from springrep@a03e8a1 tests/slab.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: springrep API replaced by the yack public API (PLAN §4.3); springrep's
// Frame::new(origin, z, x_hint) is orthonormalised here and passed as Frame::new(origin, x, y, z);
// the ball is make::revolve on Frame::at(centre) (the protocol revolves about the frame's z, with
// the profile in its xz plane), which is springrep's frame (z = −y, x = x) turned about world z;
// feature-name arguments dropped; InvalidInput is ErrorKind::Invalid.

//! Slabs: a block whose top is a B-spline height field. A field whose coefficients sit on a plane
//! is that plane, because a clamped B-spline over its Greville abscissae reproduces a linear
//! function, so these cases have closed-form volumes and areas.

use std::f64::consts::PI;

use yack::prelude::*;

const UNIFORM: [f64; 8] = [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
/// Two inner knots, spaced unevenly.
const UNEVEN: [f64; 10] = [0.0, 0.0, 0.0, 0.0, 0.3, 0.45, 1.0, 1.0, 1.0, 1.0];

fn greville(knots: &[f64], i: usize) -> f64 {
    (knots[i + 1] + knots[i + 2] + knots[i + 3]) / 3.0
}

/// Coefficients `f(u_i, v_j)` at the Greville abscissae of a cubic on `ku` × `kv`.
fn field(ku: &[f64], kv: &[f64], f: impl Fn(f64, f64) -> f64) -> Vec<Vec<f64>> {
    let (nu, nv) = (ku.len() - 4, kv.len() - 4);
    (0..nu).map(|i| (0..nv).map(|j| f(greville(ku, i), greville(kv, j))).collect()).collect()
}

/// springrep's `Frame::new(origin, z, x_hint)`: z normalised, x the hint made square to z, y = z × x.
fn springrep_frame(origin: [f64; 3], z: [f64; 3], x_hint: [f64; 3]) -> Frame {
    let z = Vec3::from(z);
    let z = z * (1.0 / z.length());
    let h = Vec3::from(x_hint);
    let x = h - z * h.dot(z);
    let x = x * (1.0 / x.length());
    Frame::new(origin, x, z.cross(x), z).unwrap()
}

fn valid(b: &Solid) {
    let issues = b.check();
    assert!(issues.is_empty(), "{issues:?}");
}

fn close(got: f64, want: f64) {
    assert!((got - want).abs() <= 1e-9 * want.abs().max(1.0), "{got} vs {want}");
}

#[test]
fn a_flat_slab_is_a_box() {
    let h = field(&UNIFORM, &UNEVEN, |_, _| 3.0);
    let s = slab(&Frame::WORLD, [10.0, 6.0], &UNIFORM, &UNEVEN, &h).unwrap();
    valid(&s);
    close(s.volume(), 180.0);
    close(s.area(), 2.0 * (60.0 + 30.0 + 18.0));
    let c = s.centroid().unwrap();
    assert!(c.distance([5.0, 3.0, 1.5].into()) <= 1e-9, "{c:?}");
}

#[test]
fn a_slab_under_a_sloping_plane_is_a_wedge_on_a_box() {
    // z = 2 + 3u over a 10 × 6 footprint: volume 6·10·3.5, a top of width 6 and slope length √109,
    // two trapezoid ends of area 35 each.
    let h = field(&UNEVEN, &UNIFORM, |u, _| 2.0 + 3.0 * u);
    let s = slab(&Frame::WORLD, [10.0, 6.0], &UNEVEN, &UNIFORM, &h).unwrap();
    valid(&s);
    close(s.volume(), 210.0);
    let area = 60.0 + 6.0 * 109f64.sqrt() + 2.0 * 35.0 + 6.0 * 2.0 + 6.0 * 5.0;
    close(s.area(), area);
}

#[test]
fn a_saddle_slab_placed_off_the_world_frame() {
    // z = 2 + 4uv: the mean height is 3, so the volume is 3·a·b wherever the frame sits.
    let h = field(&UNEVEN, &UNEVEN, |u, v| 2.0 + 4.0 * u * v);
    let frame = springrep_frame([5.0, -2.0, 1.0], [1.0, 1.0, 1.0], [1.0, -1.0, 0.0]);
    let s = slab(&frame, [8.0, 5.0], &UNEVEN, &UNEVEN, &h).unwrap();
    valid(&s);
    close(s.volume(), 120.0);
}

#[test]
fn a_ball_through_a_slab_top_joins_and_cuts() {
    let h = field(&UNEVEN, &UNEVEN, |u, v| 3.0 + 2.0 * (PI * u).sin() * v);
    let s = slab(&Frame::WORLD, [10.0, 8.0], &UNEVEN, &UNEVEN, &h).unwrap();
    let skin = Segment::Arc { center: [0.0, 0.0].into(), radius: 2.0, start: rad(-PI / 2.0), sweep: rad(PI) };
    let axis = Segment::Line { a: [0.0, 2.0].into(), b: [0.0, -2.0].into() };
    let profile = Profile::new(Loop::new(vec![skin, axis]).unwrap(), vec![]).unwrap();
    let ball = revolve(&Frame::at([4.0, 5.0, 4.0]), &profile, rad(2.0 * PI)).unwrap();
    let (u, i) = (s.union(&ball).unwrap(), s.intersection(&ball).unwrap());
    valid(&u);
    valid(&i);
    let sum = u.volume() + i.volume();
    let want = s.volume() + ball.volume();
    assert!((sum - want).abs() <= 1e-6 * want, "{sum} vs {want}");
}

#[test]
fn a_slab_that_reaches_the_floor_is_refused() {
    let mut h = field(&UNIFORM, &UNIFORM, |_, _| 3.0);
    h[1][2] = 0.0;
    let r = slab(&Frame::WORLD, [10.0, 6.0], &UNIFORM, &UNIFORM, &h);
    assert!(matches!(&r, Err(e) if matches!(e.kind, ErrorKind::Invalid)), "{:?}", r.err());
}
