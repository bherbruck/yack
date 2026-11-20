// yack: springrep module src/edit/tweak/tests.rs
// Adapted from springrep@a03e8a1 src/edit/tweak/tests.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: none (verbatim); this header added.
use super::*;
use crate::{measure, profile::Profile, sweep, topo::FaceName};

fn cube() -> Brep {
    let p = Profile::rect(Vec2::ZERO, 10.0, 10.0);
    sweep::extrude(&p, &Frame::WORLD, 10.0, "box").unwrap()
}

fn face(b: &Brep, name: &str) -> FaceId {
    let n = FaceName::new("box", name);
    b.face_ids()
        .find(|&f| b.face(f).names.contains(&n))
        .unwrap()
}

fn frame(b: &Brep, f: FaceId) -> Frame {
    plane_of(&b.face(f).surface).unwrap()
}

// The 10 mm cube's top raised 5 mm: 10 · 10 · 15.
#[test]
fn raising_the_top_stretches_the_sides() {
    let b = cube();
    let top = face(&b, "end");
    let fr = frame(&b, top).translated(Vec3::Z * 5.0);
    let out = move_planes(&b, &HashMap::from([(top, fr)])).unwrap();
    assert!((measure::volume(&out) - 1500.0).abs() < 1e-9);
    assert_eq!(out.face_count(), 6);
}

// One side tilted about its bottom edge until its top edge is 2 mm in: a wedge of 10 · 2 · 10 / 2
// comes off.
#[test]
fn tilting_a_side_about_its_foot_takes_off_a_wedge() {
    let b = cube();
    let side = face(&b, "u_max");
    let old = frame(&b, side);
    let n = old.z;
    let tilted = (n * 10.0 + Vec3::Z * 2.0).unit().unwrap();
    let foot = old.origin - Vec3::Z * old.origin.z;
    let fr = Frame::new(foot, tilted, old.x).unwrap();
    let out = move_planes(&b, &HashMap::from([(side, fr)])).unwrap();
    assert!((measure::volume(&out) - 900.0).abs() < 1e-9);
}

// A 20 × 20 × 10 block with every wall drafted about its foot: at 0.5 rad the top shrinks to
// 20 − 2·10·tan 0.5 across, and past 45° it crosses itself both ways, so its loop keeps its sense
// and only its edges show it inside out.
#[test]
fn a_draft_past_the_apex_is_refused() {
    let p = Profile::rect(Vec2::ZERO, 20.0, 20.0);
    let b = sweep::extrude(&p, &Frame::WORLD, 10.0, "box").unwrap();
    let drafted = |a: f64| {
        let mut moved = HashMap::new();
        for name in ["u_min", "u_max", "v_min", "v_max"] {
            let f = face(&b, name);
            let fr = frame(&b, f);
            let n = if b.face(f).reversed { -fr.z } else { fr.z };
            let tilted = n * a.cos() + Vec3::Z * a.sin();
            let z = if b.face(f).reversed { -tilted } else { tilted };
            let foot = fr.origin - Vec3::Z * fr.origin.z;
            moved.insert(f, Frame::new(foot, z, fr.x).unwrap());
        }
        move_planes(&b, &moved)
    };
    let top = 20.0 - 20.0 * 0.5f64.tan();
    let want = 10.0 / 3.0 * (400.0 + top * top + 20.0 * top);
    assert!((measure::volume(&drafted(0.5).unwrap()) - want).abs() < 1e-9);
    for a in [std::f64::consts::FRAC_PI_4 + 1e-6, 0.79, 0.9] {
        assert!(matches!(drafted(a), Err(Error::InvalidResult(_))), "{a}");
    }
}

#[test]
fn a_curved_face_at_a_moved_corner_is_unsupported() {
    let p = Profile::circle(Vec2::ZERO, 10.0);
    let b = sweep::extrude(&p, &Frame::WORLD, 10.0, "box").unwrap();
    let top = face(&b, "end");
    let fr = frame(&b, top).translated(Vec3::Z * 5.0);
    assert!(matches!(
        move_planes(&b, &HashMap::from([(top, fr)])),
        Err(Error::Unsupported(_))
    ));
}

// A C of 30 × 30 with 6 mm arms, 5 mm tall; a lip x 22..30, y 6..10 rises from the lower arm and
// one x 24..28, y 20..24 hangs from the upper. Raising the lower lip's tip from y 10 to 15 adds
// 8 · 5 · 5; raising it to 22 drives it through the upper lip, which no face of it shares an edge
// with, so only a test of every face against every other finds it.
#[test]
fn a_tip_moved_through_the_other_tip_is_refused() {
    let pts = [
        (0.0, 0.0),
        (30.0, 0.0),
        (30.0, 10.0),
        (22.0, 10.0),
        (22.0, 6.0),
        (6.0, 6.0),
        (6.0, 24.0),
        (24.0, 24.0),
        (24.0, 20.0),
        (28.0, 20.0),
        (28.0, 24.0),
        (30.0, 24.0),
        (30.0, 30.0),
        (0.0, 30.0),
    ]
    .map(|(x, y)| Vec2::new(x, y));
    let names: Vec<String> = (0..pts.len()).map(|i| format!("e{i}")).collect();
    let names: Vec<&str> = names.iter().map(String::as_str).collect();
    let b = sweep::extrude(&Profile::polygon(&pts, &names), &Frame::WORLD, 5.0, "box").unwrap();
    let tip = face(&b, "e2");
    let raised = |dy: f64| {
        let fr = frame(&b, tip).translated(Vec3::Y * dy);
        move_planes(&b, &HashMap::from([(tip, fr)]))
    };
    let v0 = measure::volume(&b);
    assert!((measure::volume(&raised(5.0).unwrap()) - v0 - 200.0).abs() < 1e-9);
    assert!(matches!(raised(12.0), Err(Error::InvalidResult(_))));
}

#[test]
fn a_spline_walled_prism_offset_inward_holds_steiners_area() {
    // A closed spline through 24 points of an ellipse 12 × 8, extruded 10, every face moved 1 mm
    // in. The spline's side has no exact offset of its own kind; its curve's offset is fitted and
    // extruded. By Steiner's formula an inward offset t of a smooth convex outline (t below its
    // least radius of curvature, here 8²/12 > 5) has area A − P·t + π·t², so the moved prism holds
    // (A − P + π)·8, A and P the outline's area and perimeter, read from the prism itself.
    let pts: Vec<Vec2> = (0..24)
        .map(|k| {
            let a = std::f64::consts::TAU * k as f64 / 24.0;
            Vec2::new(12.0 * a.cos(), 8.0 * a.sin())
        })
        .collect();
    let p = Profile::points(&pts, crate::profile::PointsInterp::PeriodicSpline, "wall").unwrap();
    let b = sweep::extrude(&p, &Frame::WORLD, 10.0, "can").unwrap();
    let area = measure::volume(&b) / 10.0;
    let perimeter = (measure::area(&b) - 2.0 * area) / 10.0;
    let by: HashMap<FaceId, f64> = b.face_ids().map(|f| (f, -1.0)).collect();
    let moved = offset_faces(&b, &by).unwrap();
    assert!(check::check(&moved).is_empty());
    let want = (area - perimeter + std::f64::consts::PI) * 8.0;
    let got = measure::volume(&moved);
    assert!((got - want).abs() < 1e-8 * want, "{got} for {want}");
}

#[test]
fn a_spline_sphere_octant_offset_is_the_octant_of_the_grown_sphere() {
    // The octant of the R5 sphere as a rational B-spline, offset 0.5 along its normal: the fitted
    // bicubic must lie on the R5.5 sphere about the same centre wherever it is sampled, not only at
    // the points it was checked at.
    let s = crate::geom::octant(5.0);
    let crate::geom::Surface::Nurbs(n) = &s else {
        panic!("the octant is a spline")
    };
    let ((u0, u1), (v0, v1)) = n.domain();
    let off = offset_fitted(&s, 0.5, ((u0, u1), (v0, v1))).unwrap();
    let mut worst = 0.0f64;
    for i in 0..=37 {
        for j in 0..=37 {
            let (u, v) = (
                u0 + (u1 - u0) * i as f64 / 37.0,
                v0 + (v1 - v0) * j as f64 / 37.0,
            );
            worst = worst.max((off.eval(u, v).norm() - 5.5).abs());
        }
    }
    assert!(worst <= 2.0 * OFFSET_FIT, "{worst:e}");
}

/// A box 4 wide in x, 6 deep and 4 tall, its walls in `walls` drafted about their feet by 45°:
/// the walls lean in 1 per 1 of height.
fn drafted_box(walls: &[&str]) -> Result<Brep> {
    let p = Profile::rect(Vec2::ZERO, 4.0, 6.0);
    let b = sweep::extrude(&p, &Frame::WORLD, 4.0, "box").unwrap();
    let faces: Vec<FaceId> = walls.iter().map(|w| face(&b, w)).collect();
    draft(&b, &faces, &Frame::WORLD, std::f64::consts::FRAC_PI_4)
}

// The two x walls lean in until they meet 2 up, a ridge along y: the top is gone, and the box is a
// prism on the triangle of base 4 and height 2, 6 deep: 24, of five faces.
#[test]
fn walls_drafted_until_they_meet_close_over_the_top() {
    let out = drafted_box(&["u_min", "u_max"]).unwrap();
    assert!(check::check(&out).is_empty());
    assert!((measure::volume(&out) - 24.0).abs() < 1e-9);
    assert_eq!(out.face_count(), 5);
}

// Every wall leans in: the x walls meet 2 up while the y walls are still 2 apart, a hipped roof.
// Its section at height z is (4 − 2z) by (6 − 2z), so its volume is ∫₀² (4 − 2z)(6 − 2z) dz = 56/3.
#[test]
fn all_walls_drafted_until_two_meet_make_a_hipped_roof() {
    let out = drafted_box(&["u_min", "u_max", "v_min", "v_max"]).unwrap();
    assert!(check::check(&out).is_empty());
    assert!((measure::volume(&out) - 56.0 / 3.0).abs() < 1e-9);
    assert_eq!(out.face_count(), 5);
}

// An L 10 wide and 6 tall with a step at 2 from x 4 to 10, 3 deep. Raising the step 4 brings it
// level with the top: the riser between them closes, and the L is a 10 × 6 × 3 block.
#[test]
fn a_step_raised_level_with_the_top_closes_the_riser() {
    let pts = [
        (0.0, 0.0),
        (10.0, 0.0),
        (10.0, 2.0),
        (4.0, 2.0),
        (4.0, 6.0),
        (0.0, 6.0),
    ]
    .map(|(x, y)| Vec2::new(x, y));
    let names = ["e0", "e1", "e2", "e3", "e4", "e5"];
    let b = sweep::extrude(&Profile::polygon(&pts, &names), &Frame::WORLD, 3.0, "box").unwrap();
    let out = offset_faces(&b, &HashMap::from([(face(&b, "e2"), 4.0)])).unwrap();
    assert!(check::check(&out).is_empty());
    assert!((measure::volume(&out) - 180.0).abs() < 1e-9);
    assert_eq!(out.face_count(), 6);
}

// A 10 × 8 block 6 tall with its vertical edges rounded to 1, every face moved in by 1.5: the
// rounds close to nothing and the walls meet square, a 7 × 5 × 3 block.
#[test]
fn rounds_offset_past_their_radius_close_to_sharp_corners() {
    use crate::profile::{NamedSegment, ProfileLoop, Segment};
    use std::f64::consts::{FRAC_PI_2, PI};
    let (w, d, r) = (10.0, 8.0, 1.0);
    let (x0, y0, x1, y1) = (-w / 2.0, -d / 2.0, w / 2.0, d / 2.0);
    let line = |a: (f64, f64), b: (f64, f64), n: &str| {
        NamedSegment::new(
            Segment::Line {
                a: Vec2::new(a.0, a.1),
                b: Vec2::new(b.0, b.1),
            },
            n,
        )
    };
    let arc = |c: (f64, f64), start: f64, n: &str| {
        NamedSegment::new(
            Segment::Arc {
                center: Vec2::new(c.0, c.1),
                radius: r,
                start,
                sweep: FRAC_PI_2,
            },
            n,
        )
    };
    let p = Profile {
        loops: vec![ProfileLoop {
            segments: vec![
                line((x0 + r, y0), (x1 - r, y0), "s0"),
                arc((x1 - r, y0 + r), -FRAC_PI_2, "r0"),
                line((x1, y0 + r), (x1, y1 - r), "s1"),
                arc((x1 - r, y1 - r), 0.0, "r1"),
                line((x1 - r, y1), (x0 + r, y1), "s2"),
                arc((x0 + r, y1 - r), FRAC_PI_2, "r2"),
                line((x0, y1 - r), (x0, y0 + r), "s3"),
                arc((x0 + r, y0 + r), PI, "r3"),
            ],
        }],
    };
    let b = sweep::extrude(&p, &Frame::WORLD, 6.0, "box").unwrap();
    let by: HashMap<FaceId, f64> = b.face_ids().map(|f| (f, -1.5)).collect();
    let out = offset_faces(&b, &by).unwrap();
    assert!(check::check(&out).is_empty());
    assert!((measure::volume(&out) - 105.0).abs() < 1e-9);
    assert_eq!(out.face_count(), 6);
}

// A 10 × 6 block 3 deep with a slot from x 3 to 7 down to 2: its left wall moved 4 across the slot
// lands on the right one, and the slot closes over its floor: a 10 × 6 × 3 block.
#[test]
fn a_wall_moved_across_its_slot_closes_it() {
    let pts = [
        (0.0, 0.0),
        (10.0, 0.0),
        (10.0, 6.0),
        (7.0, 6.0),
        (7.0, 2.0),
        (3.0, 2.0),
        (3.0, 6.0),
        (0.0, 6.0),
    ]
    .map(|(x, y)| Vec2::new(x, y));
    let names = ["e0", "e1", "e2", "e3", "e4", "e5", "e6", "e7"];
    let b = sweep::extrude(&Profile::polygon(&pts, &names), &Frame::WORLD, 3.0, "box").unwrap();
    let out = offset_faces(&b, &HashMap::from([(face(&b, "e5"), 4.0)])).unwrap();
    assert!(check::check(&out).is_empty());
    assert!((measure::volume(&out) - 180.0).abs() < 1e-9);
    assert_eq!(out.face_count(), 6);
}

/// A ball of radius `r` about `f.origin`, its axis `f.z`, revolved as a sketch's half disc is.
fn ball(f: &Frame, r: f64) -> Brep {
    use crate::profile::{NamedSegment, ProfileLoop, Segment};
    let seg = |segment, name: &str| NamedSegment::new(segment, name);
    let skin = Segment::Arc {
        center: Vec2::ZERO,
        radius: r,
        start: -std::f64::consts::FRAC_PI_2,
        sweep: std::f64::consts::PI,
    };
    let axis = Segment::Line {
        a: Vec2::new(0.0, r),
        b: Vec2::new(0.0, -r),
    };
    let half = Profile {
        loops: vec![ProfileLoop {
            segments: vec![seg(skin, "skin"), seg(axis, "axis")],
        }],
    };
    let meridian = Frame::new(f.origin, -f.y, f.x).unwrap();
    sweep::revolve(&half, &meridian, f.origin, f.z, TAU, "sph").unwrap()
}

/// Frames turned every which way, as moved copies are.
fn turned_frames() -> Vec<Frame> {
    (0..200)
        .map(|k| {
            let a = k as f64 * 0.731;
            let z = Vec3::new(
                a.sin() * (1.7 * a).cos(),
                a.sin() * (1.7 * a).sin(),
                a.cos(),
            );
            let x = Vec3::new((2.3 * a).cos(), (2.3 * a).sin(), 0.4);
            Frame::new(Vec3::new(1.5, -2.0, 3.0), z, x).unwrap()
        })
        .collect()
}

// A plate 12 × 8 × 3 with a hole of radius 1.8 at (6, 4), the hole's wall moved 3.4 in −x and
// 1.1 in −y, further than its radius: the hole goes with it, its rims running round it the same
// way, and the plate keeps its volume.
#[test]
fn a_hole_moved_further_than_its_radius_keeps_its_rims_turning_its_way() {
    use crate::profile::{NamedSegment, ProfileLoop, Segment};
    let mut p = Profile::rect(Vec2::new(6.0, 4.0), 12.0, 8.0);
    p.loops.push(ProfileLoop {
        segments: vec![NamedSegment::new(
            Segment::Circle {
                center: Vec2::new(6.0, 4.0),
                radius: 1.8,
                ccw: false,
            },
            "hole",
        )],
    });
    let b = sweep::extrude(&p, &Frame::WORLD, 3.0, "box").unwrap();
    let hole = face(&b, "hole");
    let moved = b
        .face(hole)
        .surface
        .transformed(&crate::math::Transform::translation(Vec3::new(
            -3.4, -1.1, 0.0,
        )));
    let out = move_faces(&b, &HashMap::from([(hole, moved)])).unwrap();
    assert!(check::check(&out).is_empty());
    let want = 3.0 * (96.0 - std::f64::consts::PI * 1.8 * 1.8);
    assert!((measure::volume(&out) - want).abs() < 1e-9 * want);
}

// A ball grown in any frame: its seam's two coedges leave the poles, where every u is the same
// point, and each must keep its own side of the period.
#[test]
fn a_ball_offset_keeps_its_seam_apart_in_any_frame() {
    let (r, d): (f64, f64) = (5.358245536894633, 0.8586557324583324);
    let want = 4.0 / 3.0 * std::f64::consts::PI * (r + d).powi(3);
    for (k, f) in turned_frames().iter().enumerate() {
        let b = ball(f, r);
        let by: HashMap<FaceId, f64> = b.face_ids().map(|f| (f, d)).collect();
        let grown = offset_faces(&b, &by).unwrap_or_else(|e| panic!("frame {k}: {e}"));
        let got = measure::volume(&grown);
        assert!(
            (got - want).abs() < 1e-6 * want,
            "frame {k}: {got} against {want}"
        );
    }
}
