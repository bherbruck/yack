// yack: T1 checks of edits and maps through the public API, against closed forms (M1.4).
#![allow(clippy::unwrap_used, clippy::disallowed_methods)] // test helpers; the facade rules are for src/
use std::f64::consts::PI;

use yack::prelude::*;

fn close(got: f64, want: f64) {
    assert!((got - want).abs() <= 1e-7 * want.abs().max(1.0), "{got} vs {want}");
}

/// The box (0, 0, 0)–(10, 20, 30).
fn brick() -> Solid {
    block(&Frame::WORLD, [10.0, 20.0, 30.0]).unwrap()
}

fn top(s: &Solid) -> FaceId {
    s.face_at([5.0, 10.0, 30.0]).unwrap()
}

#[test]
fn offset_grows_a_box_by_rounds() {
    let (a, b, c, d) = (10.0, 20.0, 30.0, 1.0);
    let s = brick().offset(d).unwrap();
    assert!(s.check().is_empty());
    let want = a * b * c + 2.0 * (a * b + b * c + c * a) * d + PI * d * d * (a + b + c) + 4.0 / 3.0 * PI * d * d * d;
    close(s.volume(), want);
}

#[test]
fn offset_faces_and_move_faces_lift_the_top() {
    let s = brick();
    let up = s.offset_faces(&[top(&s)], 2.0).unwrap();
    assert!(up.check().is_empty());
    close(up.volume(), 10.0 * 20.0 * 32.0);
    let moved = s.move_faces(&[top(&s)], &Rigid::translation([0.0, 0.0, 5.0])).unwrap();
    assert!(moved.check().is_empty());
    close(moved.volume(), 10.0 * 20.0 * 35.0);
}

#[test]
fn shell_opens_the_top() {
    let s = brick();
    let h = s.shell(&[top(&s)], 1.0).unwrap();
    assert!(h.check().is_empty());
    close(h.volume(), 10.0 * 20.0 * 30.0 - 8.0 * 18.0 * 29.0);
}

#[test]
fn shell_outward_grows_a_wall_outside() {
    let s = brick();
    let h = s.shell_outward(&[top(&s)], 1.0).unwrap();
    assert!(h.check().is_empty());
    let bb = h.bounding_box().unwrap();
    close(bb.min.x, -1.0);
    close(bb.max.x, 11.0);
    close(bb.min.z, -1.0);
    // The wall alone: less than the grown box, more than its floor.
    assert!(h.volume() > 12.0 * 22.0 && h.volume() < 12.0 * 22.0 * 31.0 - 6000.0, "{}", h.volume());
}

#[test]
fn thicken_makes_a_slab_of_a_face() {
    let s = brick();
    let t = s.thicken(&[top(&s)], 2.0).unwrap();
    assert!(t.check().is_empty());
    close(t.volume(), 10.0 * 20.0 * 2.0);
}

#[test]
fn delete_faces_fills_a_blind_hole() {
    let s = brick();
    let rod = cylinder(&Frame::at([5.0, 10.0, 20.0]), 2.0, 20.0).unwrap();
    let drilled = s.difference(&rod).unwrap();
    close(drilled.volume(), 6000.0 - PI * 4.0 * 10.0);
    let wall = drilled.face_at([7.0, 10.0, 25.0]).unwrap();
    let floor = drilled.face_at([5.0, 10.0, 20.0]).unwrap();
    let filled = drilled.delete_faces(&[wall, floor]).unwrap();
    assert!(filled.check().is_empty());
    close(filled.volume(), 6000.0);
}

#[test]
fn draft_tilts_the_sides() {
    let s = brick();
    let sides: Vec<FaceId> = [[0.0, 10.0, 15.0], [10.0, 10.0, 15.0], [5.0, 0.0, 15.0], [5.0, 20.0, 15.0]]
        .iter()
        .map(|&p| s.face_at(p).unwrap())
        .collect();
    let d = s.draft(&sides, [0.0, 0.0, 0.0], [0.0, 0.0, 1.0], deg(5.0)).unwrap();
    assert!(d.check().is_empty());
    assert!(d.volume() < s.volume(), "{}", d.volume());
}

#[test]
fn maps_scale_volume_by_their_determinant() {
    let s = brick();
    let twice = Similarity::from_rows([[2.0, 0.0, 0.0, 1.0], [0.0, 2.0, 0.0, 0.0], [0.0, 0.0, 2.0, 0.0]]).unwrap();
    close(s.transform(&twice).unwrap().volume(), 8.0 * 6000.0);
    let stretch = Affine::from_rows([[3.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0]]).unwrap();
    let a = s.transform_affine(&stretch).unwrap();
    assert!(a.check().is_empty());
    close(a.volume(), 3.0 * 6000.0);
    assert!(empty().transform(&twice).unwrap().is_empty());
}

#[test]
fn a_face_of_another_solid_is_foreign() {
    let (s, other) = (brick(), brick());
    let e = s.offset_faces(&[top(&other)], 1.0).unwrap_err();
    assert_eq!((e.kind, e.op, e.arg), (ErrorKind::ForeignId, "offset_faces", Some("faces")));
}

#[test]
fn swept_is_a_stub() {
    let e = brick().swept(&Motion::Slide { by: Vec3::new(1.0, 0.0, 0.0) }).unwrap_err();
    assert_eq!((e.kind, e.op), (ErrorKind::Unsupported, "swept"));
}
