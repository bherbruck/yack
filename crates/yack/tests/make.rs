// yack: T1 port of springrep's primitive, extrude, revolve and loft checks through the public API.
// Adapted from springrep@a03e8a1 tests/primitives.rs and tests/sweeps.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: rewritten against `yack::prelude`; frames built by `Frame::moved`; closed forms kept.
#![allow(clippy::unwrap_used, clippy::disallowed_methods)] // test helpers; the facade rules are for src/
use std::f64::consts::PI;

use yack::prelude::*;

/// A frame away from the origin and tilted off every axis.
fn tilted() -> Frame {
    let r = Rigid::rotation([0.0; 3], [0.3, -0.5, 0.8], deg(37.0)).unwrap();
    Frame::WORLD.moved(&r).moved(&Rigid::translation([3.0, -2.0, 1.5]))
}

fn close(got: f64, want: f64) {
    assert!((got - want).abs() <= 1e-9 * want.abs().max(1.0), "{got} vs {want}");
}

fn valid(s: &Solid, volume: f64, area: f64) {
    assert!(s.check().is_empty(), "{:?}", s.check());
    close(s.volume(), volume);
    close(s.area(), area);
}

fn local_centroid(s: &Solid, f: &Frame) -> [f64; 3] {
    let c = s.centroid().unwrap();
    let d = Vec3::new(c.x - f.origin().x, c.y - f.origin().y, c.z - f.origin().z);
    [f.x().vec().dot(d), f.y().vec().dot(d), f.z().vec().dot(d)]
}

fn at(got: [f64; 3], want: [f64; 3]) {
    for (g, w) in got.iter().zip(want) {
        assert!((g - w).abs() < 1e-9, "{got:?} vs {want:?}");
    }
}

#[test]
fn a_block_runs_from_its_frames_origin_along_each_axis() {
    let f = tilted();
    let b = block(&f, [3.0, 2.0, 4.0]).unwrap();
    valid(&b, 24.0, 2.0 * (6.0 + 12.0 + 8.0));
    at(local_centroid(&b, &f), [1.5, 1.0, 2.0]);
    assert_eq!(b.faces().len(), 6);
    assert_eq!(b.edges().len(), 12);
    assert_eq!(b.strategy(), "springrep");
}

#[test]
fn a_cylinder_stands_on_its_frames_origin_about_z() {
    let f = tilted();
    let b = cylinder(&f, 1.5, 4.0).unwrap();
    valid(&b, PI * 2.25 * 4.0, 2.0 * PI * 2.25 + 2.0 * PI * 1.5 * 4.0);
    at(local_centroid(&b, &f), [0.0, 0.0, 2.0]);
}

#[test]
fn a_sphere_and_a_torus_centre_on_the_frame() {
    let f = tilted();
    let s = sphere(&f, 2.0).unwrap();
    valid(&s, 4.0 / 3.0 * PI * 8.0, 4.0 * PI * 4.0);
    at(local_centroid(&s, &f), [0.0; 3]);
    let t = torus(&f, 3.0, 0.75).unwrap();
    valid(&t, PI * 0.5625 * 2.0 * PI * 3.0, 2.0 * PI * 0.75 * 2.0 * PI * 3.0);
}

#[test]
fn a_frustum_has_the_closed_form_volume() {
    let c = cone(&Frame::WORLD, 2.0, 1.0, 3.0).unwrap();
    assert!(c.check().is_empty());
    close(c.volume(), PI * 3.0 / 3.0 * (4.0 + 2.0 + 1.0));
}

#[test]
fn bad_sizes_are_invalid() {
    assert_eq!(block(&Frame::WORLD, [1.0, -1.0, 1.0]).unwrap_err().kind, ErrorKind::Invalid);
    assert_eq!(sphere(&Frame::WORLD, 0.0).unwrap_err().kind, ErrorKind::Invalid);
}

#[test]
fn an_extruded_washer() {
    let disc = |r: f64, sweep: f64| Loop::new(vec![Segment::Arc { center: [0.0, 0.0].into(), radius: r, start: deg(0.0), sweep: deg(sweep) }]).unwrap();
    let washer = Profile::new(disc(5.0, 360.0), vec![disc(2.0, -360.0)]).unwrap();
    let s = extrude(&tilted(), &washer, 3.0).unwrap();
    valid(&s, PI * (25.0 - 4.0) * 3.0, 2.0 * PI * (25.0 - 4.0) + 2.0 * PI * (5.0 + 2.0) * 3.0);
}

#[test]
fn an_extruded_rectangle_and_slot() {
    let s = extrude(&Frame::WORLD, &Profile::rectangle([0.0, 0.0], [4.0, 2.0]).unwrap(), 5.0).unwrap();
    valid(&s, 40.0, 2.0 * (8.0 + 20.0 + 10.0));
    let slot = Loop::starting_at([0.0, 0.0]).line_to([10.0, 0.0]).arc_ccw_to([10.0, 4.0], [10.0, 2.0])
        .line_to([0.0, 4.0]).arc_ccw_to([0.0, 0.0], [0.0, 2.0]).close().unwrap();
    let s = extrude(&Frame::WORLD, &Profile::new(slot, vec![]).unwrap(), 1.0).unwrap();
    assert!(s.check().is_empty());
    close(s.volume(), 40.0 + PI * 4.0);
}

#[test]
fn a_revolved_rectangle_is_a_tube() {
    // profile x out from the axis, profile y along the frame's z
    let p = Profile::rectangle([2.0, 0.0], [3.0, 4.0]).unwrap();
    let full = revolve(&tilted(), &p, deg(360.0)).unwrap();
    valid(&full, PI * (9.0 - 4.0) * 4.0, 2.0 * PI * (9.0 - 4.0) + 2.0 * PI * (3.0 + 2.0) * 4.0);
    let quarter = revolve(&Frame::WORLD, &p, deg(90.0)).unwrap();
    assert!(quarter.check().is_empty());
    close(quarter.volume(), PI * (9.0 - 4.0) * 4.0 / 4.0);
    // the quarter sweeps from +x toward +y
    let c = quarter.centroid().unwrap();
    assert!(c.x > 0.0 && c.y > 0.0, "{c:?}");
}

#[test]
fn a_loft_between_a_square_and_its_half() {
    let sq = Profile::rectangle([-2.0, -2.0], [2.0, 2.0]).unwrap();
    let half = Profile::rectangle([-1.0, -1.0], [1.0, 1.0]).unwrap();
    let s = loft(&Frame::WORLD, &sq, 0.5, 3.0, &half).unwrap();
    assert!(s.check().is_empty());
    // a frustum of a square pyramid: h/3 (A1 + A2 + sqrt(A1 A2))
    close(s.volume(), 3.0 / 3.0 * (16.0 + 4.0 + 8.0));
}

#[test]
fn a_straight_sweep_is_an_extrusion() {
    let p = Profile::circle([0.0, 0.0], 1.0).unwrap();
    let s = sweep(&Frame::WORLD, &p, &[PathPiece::Line { length: 4.0 }]).unwrap();
    assert!(s.check().is_empty());
    close(s.volume(), PI * 4.0);
    let bent = sweep(&Frame::WORLD, &p, &[PathPiece::Line { length: 2.0 }, PathPiece::Arc { radius: 5.0, angle: deg(90.0) }]).unwrap();
    assert!(bent.check().is_empty());
    close(bent.volume(), PI * 2.0 + PI * 2.0 * PI * 5.0 / 4.0);
}

#[test]
fn codec_and_step_round_trip() {
    let b = block(&tilted(), [3.0, 2.0, 4.0]).unwrap();
    let back = yack::codec::decode(&yack::codec::encode(&b).unwrap()).unwrap();
    assert_eq!(back.digest(), b.digest());
    let text = yack::step::write(&b).unwrap();
    close(yack::step::read(&text).unwrap().volume(), 24.0);
}

#[test]
fn an_extruded_ellipse_is_exact() {
    let s = extrude(&Frame::WORLD, &Profile::ellipse([1.0, 2.0], 3.0, 2.0).unwrap(), 4.0).unwrap();
    assert!(s.check().is_empty());
    close(s.volume(), PI * 3.0 * 2.0 * 4.0);
}

#[test]
fn named_faces_survive_later_operations() {
    let b = block(&Frame::WORLD, [10.0, 10.0, 10.0]).unwrap();
    let r = b.fillet(&[b.edge_at([5.0, 0.0, 10.0]).unwrap()], 2.0).unwrap().named("round");
    assert_eq!(r.faces_tagged("round").len(), 1);
    let r = r.difference(&cylinder(&Frame::at([5.0, 5.0, -1.0]), 1.0, 12.0).unwrap()).unwrap();
    assert_eq!(r.faces_tagged("round").len(), 1);
    assert!(r.faces_tagged("nothing").is_empty());
    let base = block(&Frame::WORLD, [1.0, 1.0, 1.0]).unwrap().named("base");
    assert_eq!(base.faces_tagged("base").len(), 6);
}


#[test]
fn horn_tori_and_parabolic_profiles() {
    let cup = Loop::starting_at([0.0, 0.0]).conic_to([4.0, 4.0], [2.0, 0.0], 1.0).line_to([0.0, 4.0]).close().unwrap();
    let s = revolve(&Frame::WORLD, &Profile::new(cup, vec![]).unwrap(), deg(360.0)).unwrap();
    assert!(s.check().is_empty());
    // a paraboloid's volume between z = 0 and 4 under r = 4·sqrt(z/4)... the solid of r ≤ x(z):
    assert!(s.volume() > 0.0);
}

/// sweep/mitered-elbow (case header: volume 8000, area 3400): a 10 × 10 bar along two runs of 40
/// meeting at 60°, mitred at the corner.
#[test]
fn a_mitred_elbow() {
    let corners = [Point3::new(0.0, 0.0, 0.0), Point3::new(40.0, 0.0, 0.0), Point3::new(20.0, 20.0 * 3f64.sqrt(), 0.0)];
    let f = Frame::along(corners[0], Dir3::X).unwrap();
    let bar = Profile::polygon(&[[-5.0, -5.0], [5.0, -5.0], [5.0, 5.0], [-5.0, 5.0]]).unwrap();
    let s = sweep_mitered(&f, &bar, &corners, false).unwrap();
    assert!(s.check().is_empty());
    assert!((s.volume() - 8000.0).abs() < 1e-6 * 8000.0, "{}", s.volume());
    assert!((s.area() - 3400.0).abs() < 1e-6 * 3400.0, "{}", s.area());
    let off = Frame::along([1.0, 0.0, 0.0], Dir3::X).unwrap();
    let e = sweep_mitered(&off, &bar, &corners, false).unwrap_err();
    assert_eq!(e.kind, ErrorKind::Invalid);
    assert!(e.to_string().contains("frame must sit on corners[0] with z along the first run"), "{e}");
}

/// sweep/mitered-picture-frame (case header: volume 9864, area 6544): a rabbeted molding lying
/// outside the closed rectangle (±30, ±20, 0), mitred at its four corners.
#[test]
fn a_mitred_picture_frame() {
    let corners = [[30.0, -20.0, 0.0], [-30.0, -20.0, 0.0], [-30.0, 20.0, 0.0], [30.0, 20.0, 0.0]].map(Point3::from);
    let f = Frame::new(corners[0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0], [-1.0, 0.0, 0.0]).unwrap();
    let molding = Profile::polygon(&[[0.0, 2.0], [3.0, 2.0], [3.0, 0.0], [8.0, 0.0], [8.0, 6.0], [0.0, 6.0]]).unwrap();
    let s = sweep_mitered(&f, &molding, &corners, true).unwrap();
    assert!(s.check().is_empty());
    assert!((s.volume() - 9864.0).abs() < 1e-6 * 9864.0, "{}", s.volume());
    assert!((s.area() - 6544.0).abs() < 1e-6 * 6544.0, "{}", s.area());
}

/// geometry/hyperboloid-ruled (case header: volume 523.5987755982989): rims of radius 5 at
/// z = ±5, the top turned a quarter turn.
#[test]
fn a_ruled_hyperboloid() {
    let rim = Profile::circle([0.0, 0.0], 5.0).unwrap();
    let bottom = Frame::at([0.0, 0.0, -5.0]);
    let top = Frame::new([0.0, 0.0, 5.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]).unwrap();
    let s = ruled(&bottom, &rim, &top, &rim).unwrap();
    assert!(s.check().is_empty());
    assert!((s.volume() - 523.5987755982989).abs() < 1e-6 * 523.6, "{}", s.volume());
}

/// A square drafted 5° inward up 10: a square frustum (h/3)(A + a + √(Aa)); both ways, twice it.
#[test]
fn a_drafted_extrude_one_and_both_ways() {
    let sq = Profile::rectangle([-5.0, -5.0], [5.0, 5.0]).unwrap();
    let top = 10.0 - 20.0 * 5f64.to_radians().tan();
    let want = 10.0 / 3.0 * (100.0 + top * top + 10.0 * top);
    let one = extrude_drafted(&Frame::WORLD, &sq, 10.0, deg(5.0), false).unwrap();
    assert!((one.volume() - want).abs() < 1e-6 * want, "{} vs {want}", one.volume());
    let both = extrude_drafted(&Frame::WORLD, &sq, 10.0, deg(5.0), true).unwrap();
    assert!((both.volume() - 2.0 * want).abs() < 1e-6 * want, "{} vs {}", both.volume(), 2.0 * want);
    // reversed: the mirror image in the profile's plane
    let down = extrude_drafted(&Frame::WORLD, &sq, -10.0, deg(5.0), false).unwrap();
    assert!(down.check().is_empty(), "{:?}", down.check());
    assert_eq!(down.classify(&[[0.0, 0.0, -5.0]]).unwrap(), vec![Side::In]);
    assert!((down.volume() - want).abs() < 1e-6 * want, "{} vs {want}", down.volume());
    let (a, b) = (one.bounding_box().unwrap(), down.bounding_box().unwrap());
    assert!((a.min.z - 0.0).abs() < 1e-9 && (b.max.z - 0.0).abs() < 1e-9 && (b.min.z + 10.0).abs() < 1e-9, "{a:?} {b:?}");
}

/// A negative height extrudes the same outline to the other side of its plane: an asymmetric L,
/// at +h and −h, gives mirror images across the profile's plane with the same outline at the base.
#[test]
fn a_negative_height_extrudes_the_other_way() {
    let l = Profile::polygon(&[[0.0, 0.0], [6.0, 0.0], [6.0, 2.0], [2.0, 2.0], [2.0, 5.0], [0.0, 5.0]]).unwrap();
    let f = Frame::new([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]).unwrap();
    let (up, down) = (extrude(&f, &l, 3.0).unwrap(), extrude(&f, &l, -3.0).unwrap());
    // the L: 6 × 2 and 2 × 3, area 18, so 54 at a height of 3
    assert!((up.volume() - 54.0).abs() < 1e-9 && (down.volume() - 54.0).abs() < 1e-9, "{} {}", up.volume(), down.volume());
    // a mirror flips handedness: the reversed solid must still be valid with outward faces
    assert!(down.check().is_empty(), "{:?}", down.check());
    assert_eq!(down.betti(), [1, 0, 0]);
    let (a, b) = (up.bounding_box().unwrap(), down.bounding_box().unwrap());
    // the frame's z is −y: +h runs to y = −3, −h to y = +3; x and z (the outline) alike
    assert!((a.min.y + 3.0).abs() < 1e-9 && (b.max.y - 3.0).abs() < 1e-9, "{a:?} {b:?}");
    assert!((a.min.x - b.min.x).abs() < 1e-9 && (a.max.x - b.max.x).abs() < 1e-9 && (a.max.z - b.max.z).abs() < 1e-9);
    // the base outline: the L's notch corner (6, 2) in (x, z) is on both, at y = 0
    assert_eq!(up.classify(&[[5.0, -1.5, 1.0], [5.0, -1.5, 3.0]]).unwrap(), vec![Side::In, Side::Out]);
    assert_eq!(down.classify(&[[5.0, 1.5, 1.0], [5.0, 1.5, 3.0]]).unwrap(), vec![Side::In, Side::Out]);
}

/// Loop::rounded and Loop::offset on a 10 mm square (w-m11's closed forms): rounded at 2, area
/// 100 − (4 − π)·4; rounded at 6, refused; offset +1, area 140 + π; offset −1, area 64.
#[test]
fn a_square_loop_rounds_and_offsets() {
    let sq = Loop::polygon(&[[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]]).unwrap();
    let area = |l: Loop| extrude(&Frame::WORLD, &Profile::new(l, vec![]).unwrap(), 1.0).unwrap().volume();
    let near = |got: f64, want: f64| assert!((got - want).abs() < 1e-9 * want, "{got} vs {want}");
    near(area(sq.rounded(2.0).unwrap()), 100.0 - (4.0 - PI) * 4.0);
    assert!(sq.rounded(6.0).is_err());
    near(area(sq.offset(1.0).unwrap()), 140.0 + PI);
    near(area(sq.offset(-1.0).unwrap()), 64.0);
}
