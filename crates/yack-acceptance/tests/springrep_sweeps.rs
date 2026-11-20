// yack: extrusions and revolutions against closed-form volume, area and bounding box.
// Adapted from springrep@a03e8a1 tests/sweeps.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: springrep API replaced by the yack public API (PLAN §4.3); springrep's revolve about
// world z with the profile on the frame (z = −y, x = x) is make::revolve on Frame::WORLD (the
// protocol draws the profile in the frame's xz plane and turns it about z); springrep's
// Profile::circle takes a diameter, yack's a radius; MeshOptions { linear: 0.05 } is chord 0.05;
// springrep's PeriodicSpline points profile is Loop::through; springrep face names "<id>.<face>"
// are yack (tag, Role): start/end are Role::Start/End and rect sides v_min, u_max, v_max, u_min
// are Profile::rectangle's segments s0..s3; the negative extrusion dropped and one test omitted
// (API GAP below).

//! Swept solids against closed-form volume, area and box, each checked valid and watertight.

use std::f64::consts::PI;

use yack::prelude::*;

const COARSE: f64 = 0.05;

fn assert_solid(b: &Solid, volume: f64, area: f64, lo: [f64; 3], hi: [f64; 3], rel: f64) {
    let issues = b.check();
    assert!(issues.is_empty(), "{issues:?}");
    let v = b.volume();
    let a = b.area();
    assert!((v - volume).abs() <= rel * volume, "volume {v} vs {volume}");
    assert!((a - area).abs() <= rel * area, "area {a} vs {area}");
    let bb = b.bounding_box().unwrap();
    let d = [
        bb.min.x - lo[0],
        bb.min.y - lo[1],
        bb.min.z - lo[2],
        bb.max.x - hi[0],
        bb.max.y - hi[1],
        bb.max.z - hi[2],
    ]
    .iter()
    .fold(0.0f64, |m, x| m.max(x.abs()));
    assert!(d < 1e-6, "bbox {bb:?}");
    let m = b.mesh(COARSE).unwrap();
    assert!(m.is_watertight());
    assert!((m.volume() - volume).abs() < 0.02 * volume, "mesh volume {}", m.volume());
}

fn invalid(r: &Result<Solid>) -> bool {
    matches!(r, Err(e) if matches!(e.kind, ErrorKind::Invalid))
}

fn rect(cx: f64, cy: f64, w: f64, h: f64) -> Profile {
    Profile::rectangle([cx - w / 2.0, cy - h / 2.0], [cx + w / 2.0, cy + h / 2.0]).unwrap()
}

fn line(a: [f64; 2], b: [f64; 2]) -> Segment {
    Segment::Line { a: a.into(), b: b.into() }
}

fn arc(center: [f64; 2], radius: f64, start: f64, sweep: f64) -> Segment {
    Segment::Arc { center: center.into(), radius, start: rad(start), sweep: rad(sweep) }
}

fn one_loop(segments: Vec<Segment>) -> Profile {
    Profile::new(Loop::new(segments).unwrap(), vec![]).unwrap()
}

#[test]
fn box_extrusion_is_exact_and_named() {
    let b = extrude(&Frame::WORLD, &rect(5.0, 2.0, 10.0, 4.0), 3.0).unwrap().named("blk");
    assert_solid(&b, 120.0, 2.0 * (40.0 + 30.0 + 12.0), [0.0, 0.0, 0.0], [10.0, 4.0, 3.0], 1e-12);
    // "blk.start", "blk.end", "blk.v_min", "blk.u_max", "blk.v_max", "blk.u_min".
    let side = |s: &str| Role::Side(s.into());
    for role in [Role::Start, Role::End, side("s0"), side("s1"), side("s2"), side("s3")] {
        assert_eq!(b.faces_by("blk", role.clone()).len(), 1, "{role:?}");
    }
}

#[test]
fn plate_with_a_hole_either_way_along_its_normal() {
    // The 10 × 6 rectangle about the origin less a Ø2 circle about (2, 0), the hole clockwise.
    let outer = Loop::polygon(&[[-5.0, -3.0], [5.0, -3.0], [5.0, 3.0], [-5.0, 3.0]]).unwrap();
    let hole = Loop::new(vec![arc([2.0, 0.0], 1.0, 0.0, -2.0 * PI)]).unwrap();
    let p = Profile::new(outer, vec![hole]).unwrap();
    // springrep Frame::new(0, z = −z, x = x): y = z × x = −y.
    let frame = Frame::new([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]).unwrap();
    let b = extrude(&frame, &p, 2.0).unwrap();
    let v = 2.0 * (60.0 - PI);
    let a = 2.0 * (60.0 - PI) + 2.0 * 32.0 + 2.0 * PI * 2.0;
    assert_solid(&b, v, a, [-5.0, -3.0, -2.0], [5.0, 3.0, 0.0], 1e-12);
    // API GAP: springrep also extrudes height −2 on the world frame (same solid, same checks) and
    // checks it names its start face once ("p.start"); yack's extrude refuses a negative height
    // (the protocol takes a positive one only).
}

#[test]
fn arcs_in_a_profile_sweep_cylinders_both_ways() {
    // A 10 × 10 square with a half-disc bump on the right and a half-disc bite on the left.
    // The bite's ends must meet the square's: the left side runs (0,10)→(0,8), arc, (0,2)→(0,0).
    let p = one_loop(vec![
        line([0.0, 0.0], [10.0, 0.0]),
        arc([10.0, 5.0], 5.0, -PI / 2.0, PI),
        line([10.0, 10.0], [0.0, 10.0]),
        line([0.0, 10.0], [0.0, 8.0]),
        arc([0.0, 5.0], 3.0, PI / 2.0, -PI),
        line([0.0, 2.0], [0.0, 0.0]),
    ]);
    let b = extrude(&Frame::WORLD, &p, 4.0).unwrap();
    let area2d = 100.0 + PI * 12.5 - PI * 4.5;
    let perim = 10.0 + PI * 5.0 + 10.0 + 2.0 + PI * 3.0 + 2.0;
    assert_solid(&b, 4.0 * area2d, 2.0 * area2d + 4.0 * perim, [0.0, 0.0, 0.0], [15.0, 10.0, 4.0], 1e-12);
}

#[test]
fn periodic_spline_disc_matches_its_ellipse() {
    let pts: Vec<[f64; 2]> = (0..72)
        .map(|i| {
            let a = 2.0 * PI * i as f64 / 72.0;
            [30.0 * a.cos(), 20.0 * a.sin()]
        })
        .collect();
    let p = Profile::new(Loop::through(&pts).unwrap(), vec![]).unwrap();
    let b = extrude(&Frame::WORLD, &p, 10.0).unwrap();
    assert!(b.check().is_empty());
    let v = b.volume();
    assert!((v / (PI * 6000.0) - 1.0).abs() < 2e-5, "{v}");
    assert_eq!(b.faces().len(), 3);
    let bb = b.bounding_box().unwrap();
    assert!((bb.max.x - 30.0).abs() < 1e-3 && (bb.min.y + 20.0).abs() < 1e-3);
}

#[test]
fn partial_revolve_of_a_square() {
    let p = Profile::polygon(&[[10.0, 0.0], [20.0, 0.0], [20.0, 10.0], [10.0, 10.0]]).unwrap();
    let b = revolve(&Frame::WORLD, &p, rad(PI / 2.0)).unwrap().named("rv");
    let v = PI * 300.0 * 10.0 / 4.0;
    let a = 200.0 + 2.0 * PI * 300.0 / 4.0 + 2.0 * PI * 200.0 / 4.0 + 2.0 * PI * 100.0 / 4.0;
    assert_solid(&b, v, a, [0.0, 0.0, 0.0], [20.0, 20.0, 10.0], 1e-12);
    // Six distinct face names: the four sides, the start and the end.
    let mut roles = std::collections::BTreeSet::new();
    for f in b.faces() {
        for (t, r) in b.face(f).unwrap().roles() {
            if t.as_str() == "rv" {
                roles.insert(r.clone());
            }
        }
    }
    assert_eq!(roles.len(), 6);
}

#[test]
fn partial_revolve_touching_the_axis_has_a_pole() {
    // A right triangle on the axis turned 120°: a cone sector closed by two caps.
    let p = Profile::polygon(&[[0.0, 0.0], [6.0, 0.0], [0.0, 8.0]]).unwrap();
    let b = revolve(&Frame::WORLD, &p, rad(2.0 * PI / 3.0)).unwrap();
    let v = PI * 36.0 * 8.0 / 3.0 / 3.0;
    let a = (PI * 36.0 + PI * 6.0 * 10.0) / 3.0 + 2.0 * 24.0;
    assert_solid(&b, v, a, [-3.0, 0.0, 0.0], [6.0, 6.0, 8.0], 1e-12);
}

#[test]
fn full_revolve_of_a_half_disc_is_a_sphere() {
    let p = one_loop(vec![arc([0.0, 0.0], 5.0, -PI / 2.0, PI), line([0.0, 5.0], [0.0, -5.0])]);
    let b = revolve(&Frame::WORLD, &p, rad(2.0 * PI)).unwrap();
    assert_solid(&b, 4.0 / 3.0 * PI * 125.0, 4.0 * PI * 25.0, [-5.0; 3], [5.0; 3], 1e-11);
}

#[test]
fn full_revolve_of_an_off_axis_circle_is_a_torus() {
    let p = Profile::circle([10.0, 0.0], 2.0).unwrap();
    let b = revolve(&Frame::WORLD, &p, rad(2.0 * PI)).unwrap();
    let v = 2.0 * PI * PI * 10.0 * 4.0;
    let a = 4.0 * PI * PI * 10.0 * 2.0;
    assert_solid(&b, v, a, [-12.0, -12.0, -2.0], [12.0, 12.0, 2.0], 1e-11);
}

#[test]
fn turned_stack_with_a_cone() {
    let p = Profile::polygon(&[
        [0.0, 0.0],
        [20.0, 0.0],
        [20.0, 10.0],
        [10.0, 10.0],
        [10.0, 28.0],
        [8.0, 30.0],
        [0.0, 30.0],
    ])
    .unwrap();
    let b = revolve(&Frame::WORLD, &p, rad(2.0 * PI)).unwrap();
    let frustum = PI * 2.0 / 3.0 * (100.0 + 80.0 + 64.0);
    let v = PI * 400.0 * 10.0 + PI * 100.0 * 18.0 + frustum;
    let a = 400.0 * PI + 400.0 * PI + 300.0 * PI + 360.0 * PI + 64.0 * PI + PI * 18.0 * 2.0 * 2f64.sqrt();
    assert_solid(&b, v, a, [-20.0, -20.0, 0.0], [20.0, 20.0, 30.0], 1e-12);
    assert_eq!(b.faces().len(), 6);
}

// API GAP (test revolving_a_spline_makes_a_surface_of_revolution omitted): the bulge is an open
// spline *interpolating* four points (BSpline::interpolate), and its bounding-box assert is set by
// that curve; yack has only the closed Loop::through, and Segment::Nurbs takes control points.

#[test]
fn bad_dimensions_are_invalid_input() {
    // yack: a bad width may be refused by Profile::rectangle itself, before extrude.
    let rect = |w: f64| Profile::rectangle([-w / 2.0, -0.5], [w / 2.0, 0.5]);
    let bad = [
        rect(f64::NAN).and_then(|p| extrude(&Frame::WORLD, &p, 1.0)),
        rect(1e-9).and_then(|p| extrude(&Frame::WORLD, &p, 1.0)),
        rect(1.0).and_then(|p| extrude(&Frame::WORLD, &p, f64::INFINITY)),
        rect(1.0).and_then(|p| extrude(&Frame::WORLD, &p, f64::NAN)),
    ];
    for r in bad {
        assert!(invalid(&r), "{:?}", r.err());
    }
}
