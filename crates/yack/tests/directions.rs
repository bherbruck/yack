// yack: every axis and direction parameter takes a bare Dir3, a Vec3 or an array, with no `.into()` (round 4b).
#![allow(clippy::unwrap_used, clippy::disallowed_methods)] // test helpers; the facade rules are for src/

use yack::prelude::*;

#[test]
fn a_bare_dir3_needs_no_into() {
    let _ = Motion::turn(Point3::ORIGIN, Dir3::Z, deg(0.0), deg(90.0));
    let _ = Motion::screw(Point3::ORIGIN, Dir3::Z, deg(0.0), deg(90.0), Lead::per_turn(2.0));
    let _ = Motion::slide(Dir3::X);
    let _ = Plane::new(Point3::ORIGIN, Dir3::Z).unwrap();
    let _ = Ray::new(Point3::ORIGIN, Dir3::Y).unwrap();
    let f = Frame::along(Point3::ORIGIN, Dir3::Z).unwrap();
    let b = block(&f, [1.0, 1.0, 1.0]).unwrap();
    let _ = b.project(Dir3::Z, 0.1);
}

#[test]
fn a_vec3_or_an_array_works_too() {
    let _ = Motion::turn([0.0, 0.0, 0.0], Vec3::new(0.0, 0.0, 1.0), deg(0.0), deg(90.0));
    let _ = Motion::turn([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], deg(0.0), deg(90.0));
    let _ = Frame::along([0.0, 0.0, 0.0], [0.0, 0.0, 1.0]).unwrap();
}

#[test]
fn a_zero_axis_is_refused_not_panicked() {
    assert!(Plane::new(Point3::ORIGIN, [0.0, 0.0, 0.0]).is_err());
    assert!(Frame::along(Point3::ORIGIN, Vec3::ZERO).is_err());
}

/// project: a ball seen from anywhere is one outline, nothing hidden (cadrs's exact silhouettes).
#[test]
fn a_ball_projects_to_one_outline() {
    let d = sphere(&Frame::WORLD, 5.0).unwrap().project(Dir3::Z, 0.01).unwrap();
    assert!(!d.visible.is_empty());
    assert!(d.hidden.is_empty(), "{} hidden", d.hidden.len());
}
