// yack: single-call operations (path sweeps, shell, face moves, mesh, STEP, section) against closed forms.
// Adapted from springrep@a03e8a1 tests/one_call.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: springrep API replaced by the yack public API (PLAN §4.3); springrep's sweep legs are
// PathPiece (the protocol's arc bends toward the current x, so a bend about x is the same square
// swept on a frame whose x is −y); edit::Wall::Inward is the protocol's positive shell thickness;
// MeshOptions::within(0.01) is chord 0.01; several asserts and three tests dropped (API GAP below).

//! Operations a caller once assembled from others, each as one call, against closed forms.

use std::f64::consts::{FRAC_PI_2, PI};

use yack::prelude::*;
use yack::step;

fn valid(b: &Solid) {
    let issues = b.check();
    assert!(issues.is_empty(), "{issues:?}");
}

fn invalid<T>(r: &Result<T>) -> bool {
    matches!(r, Err(e) if matches!(e.kind, ErrorKind::Invalid))
}

/// springrep's `Profile::rect(Vec2::ZERO, w, h)`.
fn rect(w: f64, h: f64) -> Profile {
    Profile::rectangle([-w / 2.0, -h / 2.0], [w / 2.0, h / 2.0]).unwrap()
}

#[test]
fn a_bend_turns_about_the_axis_it_is_given() {
    // A 2 × 2 square 5 mm up z, then a quarter turn of radius 4 about x: the path turns toward
    // x × z = −y and ends at y = −4, z = 9, heading along −y, so the solid reaches y = −4 and
    // z = 5 + 4 + 1.
    // yack: a protocol arc bends toward the frame's current x, so the frame's x is −y (y = x,
    // z = z); the 2 × 2 square about the origin is the same square on that frame.
    let p = rect(2.0, 2.0);
    let frame = Frame::new([0.0, 0.0, 0.0], [0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]).unwrap();
    let legs = [PathPiece::Line { length: 5.0 }, PathPiece::Arc { radius: 4.0, angle: rad(FRAC_PI_2) }];
    let b = sweep(&frame, &p, &legs).unwrap();
    valid(&b);
    let want = 4.0 * (5.0 + 4.0 * FRAC_PI_2);
    assert!((b.volume() - want).abs() < 1e-9 * want);
    let bb = b.bounding_box().unwrap();
    assert!((bb.min.y + 4.0).abs() < 1e-9 && (bb.max.y - 1.0).abs() < 1e-9);
    assert!((bb.max.z - 10.0).abs() < 1e-9 && (bb.max.x - 1.0).abs() < 1e-9);
    // API GAP: springrep refuses an arc about the path's own direction (axis z); a §4.3 PathPiece
    // has no axis, so the askew bend cannot be stated.
}

// API GAP (test a_sharp_turn_is_filled_by_the_outside_of_the_profile_turned_about_the_corner
// omitted): it needs a turn on the spot (springrep Leg::Turn, a bend of radius 0); §4.3 PathPiece
// has only Line and Arc, and the protocol requires every bend radius to be positive.

#[test]
fn a_sweep_along_lines_and_an_arc_has_its_profile_s_area_times_the_path() {
    // A 2 × 1 rectangle centred on the path: 5 mm up z, a quarter turn of radius 4 toward x, 3 mm
    // on along x. By Pappus the volume is the area times the path's length.
    let p = rect(2.0, 1.0);
    let legs = [
        PathPiece::Line { length: 5.0 },
        PathPiece::Arc { radius: 4.0, angle: rad(FRAC_PI_2) },
        PathPiece::Line { length: 3.0 },
    ];
    let b = sweep(&Frame::WORLD, &p, &legs).unwrap();
    valid(&b);
    let length = 5.0 + 4.0 * FRAC_PI_2 + 3.0;
    let v = b.volume();
    assert!((v - 2.0 * length).abs() < 1e-9, "{v}");
    // The bend ends 4 out and 4 up from its start, the profile 1 to either side; 3 more along x.
    let hi = b.bounding_box().unwrap().max;
    assert!((hi.x - 7.0).abs() < 1e-9 && (hi.z - 10.0).abs() < 1e-9, "{hi:?}");
}

#[test]
fn a_sweep_along_no_legs_is_invalid() {
    let p = rect(2.0, 1.0);
    assert!(invalid(&sweep(&Frame::WORLD, &p, &[])));
}

#[test]
fn a_path_sweep_along_a_straight_path_is_a_prism_or_a_twisted_one() {
    // A 2 × 1 rectangle 6 mm along z: a line, and a cubic whose control points lie on that line.
    // yack: the line is the degree-1 spline from (0, 0, 0) to (0, 0, 6) (springrep: the line
    // through the origin along z over [0, 6]).
    let p = rect(2.0, 1.0);
    let line = Spline3::bezier(&[[0.0, 0.0, 0.0], [0.0, 0.0, 6.0]]).unwrap();
    let ctrl: Vec<Point3> = [0.0, 1.0, 4.0, 6.0].iter().map(|&z| [0.0, 0.0, z].into()).collect();
    let knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
    let cubic = Spline3::new(3, knots, ctrl).unwrap();
    for path in [&line, &cubic] {
        let flat = sweep_path(&Frame::WORLD, &p, path, rad(0.0)).unwrap();
        valid(&flat);
        assert_eq!(flat.faces().len(), 6);
        let (v, a) = (flat.volume(), flat.area());
        assert!((v - 12.0).abs() < 1e-9 && (a - 40.0).abs() < 1e-9, "{v} {a}");
        // Twisted a quarter turn, each section is still the rectangle: the volume is unchanged.
        let twisted = sweep_path(&Frame::WORLD, &p, path, rad(FRAC_PI_2)).unwrap();
        valid(&twisted);
        let v = twisted.volume();
        assert!((v - 12.0).abs() < 1e-7, "{v}");
        // Its top is the rectangle turned: 1 wide along x and 2 along y.
        let top = twisted.face_at([0.0, 0.9, 6.0]).unwrap();
        assert!(twisted.face_at([0.9, 0.0, 6.0]).map_or(true, |f| f != top));
    }
}

#[test]
fn a_shell_open_at_the_top_keeps_walls_of_its_thickness() {
    // A 10 × 8 × 6 block hollowed to 1 mm walls, open at the top: the block less a 8 × 6 × 5
    // pocket.
    let blk = block(&Frame::WORLD, [10.0, 8.0, 6.0]).unwrap();
    let top = blk.face_at([5.0, 4.0, 6.0]).unwrap();
    let b = blk.shell(&[top], 1.0).unwrap();
    valid(&b);
    let v = b.volume();
    assert!((v - (480.0 - 240.0)).abs() < 1e-9, "{v}");
    // A wall as thick as half the part leaves no hollow, and is refused.
    assert!(blk.shell(&[top], 4.0).is_err());
}

#[test]
fn faces_moved_by_a_motion_go_where_it_takes_them() {
    // The top of a 10 × 8 × 6 block lifted 2 mm: a 10 × 8 × 8 block.
    let blk = block(&Frame::WORLD, [10.0, 8.0, 6.0]).unwrap();
    let top = blk.face_at([5.0, 4.0, 6.0]).unwrap();
    let lift = Rigid::translation([0.0, 0.0, 2.0]);
    let b = blk.move_faces(&[top], &lift).unwrap();
    valid(&b);
    let v = b.volume();
    assert!((v - 640.0).abs() < 1e-9, "{v}");
    // A map that stretches is no motion.
    // yack: a Rigid cannot hold a stretch, so the refusal comes from Rigid::from_rows, before
    // move_faces.
    let stretch = Rigid::from_rows([[2.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0]]);
    assert!(invalid(&stretch));
}

// API GAP (test a_conic_segment_is_the_ellipse_arc_it_lies_on omitted): it evaluates a 2-D conic
// segment (at(t), start(), end()), reads back its ellipse (center, rotation, major, minor), and
// expects weights ≥ 1 or 0 to be refused on their own; §4.3 Segment::Conic is a plain enum
// variant with no constructor, evaluator or ellipse form.

// API GAP (test a_file_of_two_touching_solids_reads_as_the_one_part omitted): it builds one value
// holding two touching solids without merging them (springrep Brep::append, solid_ids().count()
// == 2) and writes that as a two-solid STEP file; §4.3 has no way to gather solids into a set
// without a boolean. The read-back half (one piece, volume 96) would be step::read + betti()[0].

#[test]
fn a_mesh_within_a_tolerance_lies_that_close() {
    // A ball of radius 5 meshed within 0.01: the mesh's volume falls short of the ball's by no
    // more than the tolerance times its area.
    let ball = sphere(&Frame::WORLD, 5.0).unwrap();
    let m = ball.mesh(0.01).unwrap();
    assert!(m.is_watertight());
    let exact = 4.0 / 3.0 * PI * 125.0;
    assert!((m.volume() - exact).abs() <= 0.01 * 4.0 * PI * 25.0, "{}", m.volume());
}

#[test]
fn the_step_writer_says_which_order_it_wrote_the_faces_in() {
    let b = extrude(&Frame::WORLD, &rect(4.0, 2.0), 3.0).unwrap();
    let text = step::write(&b).unwrap();
    let written: Vec<&str> = text.split("ADVANCED_FACE(").skip(1).collect();
    assert_eq!(written.len(), 6);
    // API GAP: springrep's step::write_faces also returns the faces in the order written, and
    // each ADVANCED_FACE carries that face's name; §4.3 step::write returns only the text and
    // faces have no readable names.
    assert_eq!(text, step::write(&b).unwrap());
}

#[test]
fn a_section_is_given_as_the_curves_it_runs_along() {
    // A Ø10 rod cut square to its axis: one outline, every piece of it a circle of radius 5
    // about the axis at the cut's height.
    let rod = cylinder(&Frame::WORLD, 5.0, 8.0).unwrap();
    let plane = Plane::new([0.0, 0.0, 3.0], [0.0, 0.0, 1.0]).unwrap();
    // yack: Solid::section gives polylines within a chord; 0.01 is chosen here (springrep's
    // section_curves gives exact curves and takes no tolerance).
    let outlines = rod.section(&plane, 0.01).unwrap();
    assert_eq!(outlines.len(), 1);
    assert!(!outlines[0].is_empty());
    // API GAP: each piece is a Circle of radius 5 centred at z = 3, and the pieces turn 2π in
    // all; §4.3 section returns points, not the curves they lie on.
}
