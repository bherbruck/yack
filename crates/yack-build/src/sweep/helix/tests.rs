// yack: springrep module src/sweep/helix/tests.rs
// Adapted from springrep@a03e8a1 src/sweep/helix/tests.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/sweep/helix/tests.rs to yack-build (M3 W7); paths and visibility only.
use std::f64::consts::{PI, TAU};
use super::*;
use crate::measure;

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs()
}

/// The xz plane through the point 10 mm out along x, its normal along y: a meridian of the z axis.
fn meridian() -> Frame {
    Frame::new(Vec3::new(10.0, 0.0, 0.0), Vec3::Y, -Vec3::X).unwrap()
}

fn about_z(pitch: f64, sign: f64) -> Helix {
    Helix {
        origin: Vec3::ZERO,
        axis: Vec3::Z,
        rise: pitch / TAU,
        sign,
    }
}

#[test]
fn a_circle_screwed_a_quarter_turn_holds_pappus_volume() {
    let p = Profile::circle(Vec2::ZERO, 2.0);
    let b = helix(&p, &meridian(), &about_z(8.0, 1.0), PI / 2.0, "coil").unwrap();
    assert_eq!(b.face_count(), 3);
    assert!(rel(measure::volume(&b), 5.0 * PI * PI) < 1e-9);
    let bb = measure::bbox(&b);
    assert!((bb.min.z + 1.0).abs() < 1e-6 && (bb.max.z - 3.0).abs() < 1e-6);
    let names = b.name_counts();
    assert_eq!(names["coil.start"], 1);
    assert_eq!(names["coil.end"], 1);
}

#[test]
fn a_profile_drawn_on_a_frame_at_the_axis_is_screwed() {
    // The circle above, its frame's origin on the axis and the circle drawn 10 mm out along the
    // frame's x (world -x, so at world +x 10).
    let at_axis = Frame::new(Vec3::ZERO, Vec3::Y, -Vec3::X).unwrap();
    let p = Profile::circle(Vec2::new(-10.0, 0.0), 2.0);
    let b = helix(&p, &at_axis, &about_z(8.0, 1.0), PI / 2.0, "coil").unwrap();
    assert!(
        rel(measure::volume(&b), 5.0 * PI * PI) < 1e-9,
        "{}",
        measure::volume(&b)
    );
}

#[test]
fn a_square_screwed_left_handed_keeps_its_sides() {
    let p = Profile::rect(Vec2::ZERO, 2.0, 2.0);
    let b = helix(&p, &meridian(), &about_z(8.0, -1.0), PI / 2.0, "thread").unwrap();
    assert_eq!(b.face_count(), 6);
    assert!(rel(measure::volume(&b), 20.0 * PI) < 1e-9);
}

#[test]
fn a_profile_on_the_axis_is_refused() {
    let p = Profile::circle(Vec2::new(10.0, 0.0), 2.0);
    let e = helix(&p, &meridian(), &about_z(5.0, 1.0), TAU, "coil");
    assert!(matches!(e, Err(Error::InvalidInput(_))));
}

#[test]
fn whole_and_half_turns_keep_their_faces_outward() {
    let p = Profile::circle(Vec2::ZERO, 2.0);
    for turns in [0.5, 1.0, 3.0] {
        let b = helix(&p, &meridian(), &about_z(5.0, 1.0), turns * TAU, "coil").unwrap();
        let want = PI * turns * TAU * 10.0;
        assert!(rel(measure::volume(&b), want) < 1e-9, "{turns}");
    }
}

#[test]
fn coils_that_meet_are_refused() {
    let p = Profile::circle(Vec2::ZERO, 2.0);
    for pitch in [1.5, 2.0, 2.0 + 1e-7] {
        let e = helix(&p, &meridian(), &about_z(pitch, 1.0), 2.0 * TAU, "coil");
        assert!(matches!(e, Err(Error::InvalidInput(_))), "{pitch}");
    }
    // Under a whole turn the copy a pitch along never comes round.
    assert!(helix(&p, &meridian(), &about_z(1.5, 1.0), 0.9 * TAU, "coil").is_ok());
    let b = helix(&p, &meridian(), &about_z(2.1, 1.0), 2.0 * TAU, "coil").unwrap();
    assert!(rel(measure::volume(&b), PI * 2.0 * TAU * 10.0) < 1e-9);
}

#[test]
fn a_profile_across_the_axis_is_refused() {
    let frame = Frame::new(Vec3::new(0.8, 0.0, 0.0), Vec3::Y, -Vec3::X).unwrap();
    let p = Profile::circle(Vec2::ZERO, 2.0);
    let e = helix(&p, &frame, &about_z(5.0, 1.0), PI, "coil");
    assert!(matches!(e, Err(Error::InvalidInput(_))));
}

#[test]
fn a_twisted_extrusion_holds_its_section_s_area_all_the_way_up() {
    let rise = 5.0;
    let twist = 0.6;
    let motion = about_z(rise / twist * TAU, 1.0);
    let rect = Profile::rect(Vec2::ZERO, 2.0, 3.0);
    let b = helix(&rect, &Frame::WORLD, &motion, twist, "tw").unwrap();
    assert!(check::check(&b).is_empty());
    assert!(rel(measure::volume(&b), 6.0 * rise) < 1e-7);
    let disc = Profile::circle(Vec2::new(3.0, 1.0), 2.0);
    let b = helix(
        &disc,
        &Frame::WORLD,
        &about_z(rise / twist * TAU, -1.0),
        twist,
        "tw",
    )
    .unwrap();
    assert!(check::check(&b).is_empty());
    assert!(rel(measure::volume(&b), PI * rise) < 1e-7);
}
