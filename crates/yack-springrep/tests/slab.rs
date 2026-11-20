// yack: Slabs: a block whose top is a B-spline height field. A field whose coefficients sit on a plane
// Adapted from springrep@a03e8a1 tests/slab.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: crate paths springrep:: -> yack_springrep::; this header added.
//! Slabs: a block whose top is a B-spline height field. A field whose coefficients sit on a plane
//! is that plane, because a clamped B-spline over its Greville abscissae reproduces a linear
//! function, so these cases have closed-form volumes and areas.

use std::f64::consts::PI;

use yack_springrep::{
    Error,
    boolean::{intersection, union},
    check,
    math::{Frame, Vec2, Vec3},
    measure,
    primitive::slab,
    profile::{NamedSegment, Profile, ProfileLoop, Segment},
    sweep::revolve,
    topo::Brep,
};

const UNIFORM: [f64; 8] = [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
/// Two inner knots, spaced unevenly.
const UNEVEN: [f64; 10] = [0.0, 0.0, 0.0, 0.0, 0.3, 0.45, 1.0, 1.0, 1.0, 1.0];

fn greville(knots: &[f64], i: usize) -> f64 {
    (knots[i + 1] + knots[i + 2] + knots[i + 3]) / 3.0
}

/// Coefficients `f(u_i, v_j)` at the Greville abscissae of a cubic on `ku` × `kv`.
fn field(ku: &[f64], kv: &[f64], f: impl Fn(f64, f64) -> f64) -> Vec<Vec<f64>> {
    let (nu, nv) = (ku.len() - 4, kv.len() - 4);
    (0..nu)
        .map(|i| {
            (0..nv)
                .map(|j| f(greville(ku, i), greville(kv, j)))
                .collect()
        })
        .collect()
}

fn valid(b: &Brep) {
    let issues = check::check(b);
    assert!(issues.is_empty(), "{issues:?}");
}

fn close(got: f64, want: f64) {
    assert!(
        (got - want).abs() <= 1e-9 * want.abs().max(1.0),
        "{got} vs {want}"
    );
}

#[test]
fn a_flat_slab_is_a_box() {
    let h = field(&UNIFORM, &UNEVEN, |_, _| 3.0);
    let s = slab(&Frame::WORLD, [10.0, 6.0], (&UNIFORM, &UNEVEN), &h, "s").unwrap();
    valid(&s);
    close(measure::volume(&s), 180.0);
    close(measure::area(&s), 2.0 * (60.0 + 30.0 + 18.0));
    let c = measure::mass_properties(&s).centre;
    assert!(c.dist(Vec3::new(5.0, 3.0, 1.5)) <= 1e-9, "{c:?}");
}

#[test]
fn a_slab_under_a_sloping_plane_is_a_wedge_on_a_box() {
    // z = 2 + 3u over a 10 × 6 footprint: volume 6·10·3.5, a top of width 6 and slope length √109,
    // two trapezoid ends of area 35 each.
    let h = field(&UNEVEN, &UNIFORM, |u, _| 2.0 + 3.0 * u);
    let s = slab(&Frame::WORLD, [10.0, 6.0], (&UNEVEN, &UNIFORM), &h, "s").unwrap();
    valid(&s);
    close(measure::volume(&s), 210.0);
    let area = 60.0 + 6.0 * 109f64.sqrt() + 2.0 * 35.0 + 6.0 * 2.0 + 6.0 * 5.0;
    close(measure::area(&s), area);
}

#[test]
fn a_saddle_slab_placed_off_the_world_frame() {
    // z = 2 + 4uv: the mean height is 3, so the volume is 3·a·b wherever the frame sits.
    let h = field(&UNEVEN, &UNEVEN, |u, v| 2.0 + 4.0 * u * v);
    let frame = Frame::new(
        Vec3::new(5.0, -2.0, 1.0),
        Vec3::new(1.0, 1.0, 1.0),
        Vec3::new(1.0, -1.0, 0.0),
    )
    .unwrap();
    let s = slab(&frame, [8.0, 5.0], (&UNEVEN, &UNEVEN), &h, "s").unwrap();
    valid(&s);
    close(measure::volume(&s), 120.0);
}

#[test]
fn a_ball_through_a_slab_top_joins_and_cuts() {
    let h = field(&UNEVEN, &UNEVEN, |u, v| 3.0 + 2.0 * (PI * u).sin() * v);
    let s = slab(&Frame::WORLD, [10.0, 8.0], (&UNEVEN, &UNEVEN), &h, "s").unwrap();
    let skin = NamedSegment::new(
        Segment::Arc {
            center: Vec2::ZERO,
            radius: 2.0,
            start: -PI / 2.0,
            sweep: PI,
        },
        "skin",
    );
    let axis = NamedSegment::new(
        Segment::Line {
            a: Vec2::new(0.0, 2.0),
            b: Vec2::new(0.0, -2.0),
        },
        "axis",
    );
    let profile = Profile {
        loops: vec![ProfileLoop {
            segments: vec![skin, axis],
        }],
    };
    let c = Vec3::new(4.0, 5.0, 4.0);
    let frame = Frame::new(c, Vec3::new(0.0, -1.0, 0.0), Vec3::X).unwrap();
    let ball = revolve(&profile, &frame, c, Vec3::Z, 2.0 * PI, "b").unwrap();
    let (u, i) = (union(&s, &ball).unwrap(), intersection(&s, &ball).unwrap());
    valid(&u);
    valid(&i);
    let sum = measure::volume(&u) + measure::volume(&i);
    let want = measure::volume(&s) + measure::volume(&ball);
    assert!((sum - want).abs() <= 1e-6 * want, "{sum} vs {want}");
}

#[test]
fn a_slab_that_reaches_the_floor_is_refused() {
    let mut h = field(&UNIFORM, &UNIFORM, |_, _| 3.0);
    h[1][2] = 0.0;
    let r = slab(&Frame::WORLD, [10.0, 6.0], (&UNIFORM, &UNIFORM), &h, "s");
    assert!(matches!(r, Err(Error::InvalidInput(_))), "{r:?}");
}
