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
    let t = s.thicken(&[top(&s)], 2.0, Join::Round).unwrap();
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
fn swept_sequences_go_leg_by_leg() {
    let one = brick().swept(&Motion::slide([1.0, 0.0, 0.0])).unwrap();
    let two = brick()
        .swept(&Motion::then(vec![Motion::slide([1.0, 0.0, 0.0]), Motion::slide([1.0, 0.0, 0.0])]))
        .unwrap();
    assert!(two.volume() > one.volume());
    let e = brick().swept(&Motion::then(vec![Motion::then(vec![])])).unwrap_err();
    assert_eq!((e.kind, e.op), (ErrorKind::Unsupported, "swept"));
}

#[test]
fn a_rounded_shell_holds_more_than_a_sharp_one_outward() {
    let b = block(&Frame::WORLD, [10.0, 10.0, 10.0]).unwrap();
    let top = b.face_at([5.0, 5.0, 10.0]).unwrap();
    match (b.shell_outward(&[top], 1.0), b.shell_outward_rounded(&[top], 1.0)) {
        (Ok(sharp), Ok(round)) => assert!(round.volume() < sharp.volume()),
        (_, Err(e)) => assert_eq!(e.kind, ErrorKind::Unsupported, "{e}"),
        (Err(e), _) => panic!("{e}"),
    }
}

#[test]
fn offset_and_shell_faces_take_the_name_given() {
    let b = block(&Frame::WORLD, [10.0, 10.0, 10.0]).unwrap();
    let grown = b.offset(1.0).unwrap().named("grown");
    assert!(!grown.faces_tagged("grown").is_empty());
    let top = b.face_at([5.0, 5.0, 10.0]).unwrap();
    let hollow = b.shell(&[top], 1.0).unwrap().named("hollow");
    assert!(!hollow.faces_by("hollow", Role::Inner).is_empty(), "strategy {}", hollow.strategy());
}


#[test]
fn turns_together_and_from_a_start_angle() {
    let b = brick();
    let quarter = b.swept(&Motion::turn([0.0; 3], [0.0, 0.0, 1.0], deg(0.0), deg(90.0)));
    let later = b.swept(&Motion::turn([0.0; 3], [0.0, 0.0, 1.0], deg(90.0), deg(180.0)));
    if let (Ok(q), Ok(l)) = (&quarter, &later) {
        assert!((q.volume() - l.volume()).abs() < 1e-6 * q.volume(), "a quarter turn from 90° holds as much as one from 0°");
    }
    match b.swept(&Motion::together(vec![Motion::slide([2.0, 0.0, 0.0]), Motion::slide([0.0, 2.0, 0.0])])) {
        Ok(s) => assert!(s.volume() > b.volume() && s.check().is_empty()),
        Err(e) => assert_eq!(e.kind, ErrorKind::Unsupported, "{e}"),
    }
    assert_eq!(sewn(&[]).unwrap_err().kind, ErrorKind::Invalid);
    // a unit cube from five bilinear patches and a filled floor
    let p = |a: [f64; 3], b: [f64; 3], c: [f64; 3], d: [f64; 3]| Patch::Bezier {
        ctrl: vec![vec![a.into(), b.into()], vec![d.into(), c.into()]],
    };
    let line = |a: [f64; 3], b: [f64; 3]| Spline3::bezier(&[a, b]).unwrap();
    let (o, x, xy, y) = ([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0]);
    let up = |q: [f64; 3]| [q[0], q[1], 1.0];
    let cube = sewn(&[
        p(up(o), up(x), up(xy), up(y)),
        p(o, x, up(x), up(o)),
        p(x, xy, up(xy), up(x)),
        p(xy, y, up(y), up(xy)),
        p(y, o, up(o), up(y)),
        Patch::PlanarFill { boundary: vec![line(o, x), line(x, xy), line(xy, y), line(y, o)] },
    ]);
    let c = cube.unwrap();
    assert!((c.volume() - 1.0).abs() < 1e-6, "{}", c.volume());
}

/// swept/tumbling-cube: cadrs sweeps a cube under two turns at once exactly (its polyhedron
/// path), so the answer is kept: exact-only refuses only where cadrs would mesh.
#[test]
fn a_tumbling_cube_is_swept_exactly() {
    let cube = block(&Frame::at([2.4, -0.6, 1.4]), [1.2, 1.2, 1.2]).unwrap();
    let roll = Motion::turn(Point3::new(2.0, 0.0, 0.0), Dir3::Z, deg(-18.75), deg(18.75));
    let tumble = Motion::turn(Point3::new(0.0, 0.5, 0.5), Dir3::X, deg(75.0), deg(-75.0));
    let s = cube.swept(&Motion::together(vec![roll, tumble])).unwrap();
    assert!(s.is_exact() && s.check().is_empty());
    assert!(s.volume() > cube.volume());
}

/// The meshed control: a cube climbing a screw whose lead grows is one cadrs sweeps only as a
/// mesh; it is refused, never answered faceted.
#[test]
fn a_climb_cadrs_only_meshes_is_refused() {
    let cube = block(&Frame::at([2.0, -0.5, 0.0]), [1.0, 1.0, 1.0]).unwrap();
    let climb = Motion::screw(Point3::ORIGIN, Dir3::Z, rad(0.0), rad(1.0), Lead::polynomial(vec![0.0, 0.5, 0.5]));
    match cube.swept(&climb) {
        Ok(s) => panic!("answered, strategy {}, volume {}", s.strategy(), s.volume()),
        Err(e) => assert_ne!(e.kind, ErrorKind::Internal, "{e}"),
    }
}

/// thicken/angle-bracket-{sharp,rounded}: the floor and one wall of a block, thickened outward
/// together; the crease between them mitred (168) or rounded by a quarter cylinder (166.28…).
#[test]
fn a_bracket_thickens_sharp_or_round() {
    let b = block(&Frame::WORLD, [10.0, 8.0, 10.0]).unwrap();
    let sheet = b.faces_at(&[[5.0, 4.0, 0.0], [0.0, 4.0, 5.0]]).unwrap();
    close(b.thicken(&sheet, 1.0, Join::Sharp).unwrap().volume(), 168.0);
    close(b.thicken(&sheet, 1.0, Join::Round).unwrap().volume(), 160.0 + 2.0 * PI);
}
