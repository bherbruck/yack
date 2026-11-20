// yack: the five primitive solids against closed-form volume and area, on a tilted frame.
// Adapted from springrep@a03e8a1 tests/primitives.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: springrep API replaced by the yack public API (PLAN §4.3); springrep's
// Frame::new(origin, z, x_hint) is orthonormalised here and passed as Frame::new(origin, x, y, z);
// feature-name arguments dropped; InvalidInput is ErrorKind::Invalid.

//! The five primitive solids against closed-form volume and area, each checked valid, on a frame
//! away from the origin and tilted off every axis.

use std::f64::consts::PI;

use yack::prelude::*;

/// springrep's `Frame::new(origin, z, x_hint)`: z normalised, x the hint made square to z, y = z × x.
/// Returns the axes too, so the tests can read coordinates in the frame.
fn springrep_frame(origin: [f64; 3], z: [f64; 3], x_hint: [f64; 3]) -> (Frame, [Vec3; 3]) {
    let z = Vec3::from(z);
    let z = z * (1.0 / z.length());
    let h = Vec3::from(x_hint);
    let x = h - z * h.dot(z);
    let x = x * (1.0 / x.length());
    let y = z.cross(x);
    (Frame::new(origin, x, y, z).unwrap(), [x, y, z])
}

fn tilted() -> (Frame, [f64; 3], [Vec3; 3]) {
    let origin = [3.0, -2.0, 1.5];
    let (f, axes) = springrep_frame(origin, [0.3, -0.5, 0.8], [1.0, 0.2, 0.0]);
    (f, origin, axes)
}

fn assert_solid(b: &Solid, volume: f64, area: f64) {
    let issues = b.check();
    assert!(issues.is_empty(), "{issues:?}");
    let (v, a) = (b.volume(), b.area());
    assert!((v - volume).abs() <= 1e-10 * volume, "volume {v} vs {volume}");
    assert!((a - area).abs() <= 1e-10 * area, "area {a} vs {area}");
}

/// The centroid in the frame's own coordinates: where along its axes the solid sits.
fn local_centroid(b: &Solid, origin: [f64; 3], axes: [Vec3; 3]) -> [f64; 3] {
    let c = b.centroid().unwrap();
    let d = Vec3::new(c.x - origin[0], c.y - origin[1], c.z - origin[2]);
    [d.dot(axes[0]), d.dot(axes[1]), d.dot(axes[2])]
}

fn assert_at(got: [f64; 3], want: [f64; 3]) {
    let d = (0..3).map(|i| (got[i] - want[i]).abs()).fold(0.0, f64::max);
    assert!(d < 1e-9, "{got:?}");
}

fn refused(r: Result<Solid>) -> bool {
    matches!(r, Err(e) if matches!(e.kind, ErrorKind::Invalid))
}

#[test]
fn a_block_runs_from_its_frames_origin_along_each_axis() {
    let (f, o, axes) = tilted();
    let b = block(&f, [3.0, 2.0, 4.0]).unwrap();
    assert_solid(&b, 24.0, 2.0 * (6.0 + 12.0 + 8.0));
    assert_at(local_centroid(&b, o, axes), [1.5, 1.0, 2.0]);
}

#[test]
fn a_cylinder_stands_on_its_frames_origin_about_z() {
    let (f, o, axes) = tilted();
    let b = cylinder(&f, 1.5, 4.0).unwrap();
    assert_solid(&b, PI * 2.25 * 4.0, 2.0 * PI * 2.25 + 2.0 * PI * 1.5 * 4.0);
    assert_at(local_centroid(&b, o, axes), [0.0, 0.0, 2.0]);
}

#[test]
fn a_sphere_is_centred_on_its_frames_origin() {
    let (f, o, axes) = tilted();
    let b = sphere(&f, 2.0).unwrap();
    assert_solid(&b, 4.0 / 3.0 * PI * 8.0, 4.0 * PI * 4.0);
    assert_at(local_centroid(&b, o, axes), [0.0; 3]);
}

#[test]
fn a_torus_rings_its_frames_z_axis() {
    let (f, o, axes) = tilted();
    let b = torus(&f, 3.0, 0.75).unwrap();
    let tube = PI * 0.75 * 0.75;
    assert_solid(&b, tube * 2.0 * PI * 3.0, 2.0 * PI * 0.75 * 2.0 * PI * 3.0);
    assert_at(local_centroid(&b, o, axes), [0.0; 3]);
}

#[test]
fn a_cone_is_a_frustum_or_comes_to_an_apex_at_either_end() {
    let (f, _, _) = tilted();
    let frustum = |r0: f64, r1: f64, h: f64| {
        let slant = ((r0 - r1).powi(2) + h * h).sqrt();
        (PI * h / 3.0 * (r0 * r0 + r0 * r1 + r1 * r1), PI * (r0 * r0 + r1 * r1 + (r0 + r1) * slant))
    };
    for (r0, r1) in [(2.0, 0.5), (2.0, 0.0), (0.0, 1.5)] {
        let b = cone(&f, r0, r1, 3.0).unwrap();
        let (v, a) = frustum(r0, r1, 3.0);
        assert_solid(&b, v, a);
    }
    // Equal radii are the frustum that is a cylinder.
    let b = cone(&f, 1.5, 1.5, 4.0).unwrap();
    assert_solid(&b, PI * 2.25 * 4.0, 2.0 * PI * 2.25 + 2.0 * PI * 1.5 * 4.0);
}

#[test]
fn a_size_at_or_below_zero_is_invalid_input() {
    let (f, _, _) = tilted();
    assert!(refused(block(&f, [3.0, 0.0, 4.0])));
    assert!(refused(cylinder(&f, -1.0, 4.0)));
    assert!(refused(cylinder(&f, 1.0, 0.0)));
    assert!(refused(sphere(&f, 0.0)));
    assert!(refused(cone(&f, 0.0, 0.0, 3.0)));
    assert!(refused(cone(&f, 1.0, -0.5, 3.0)));
    // A tube as wide as its ring crosses the axis.
    assert!(refused(torus(&f, 1.0, 1.0)));
    assert!(refused(torus(&f, 1.0, 1.5)));
}
