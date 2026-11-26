// yack: Booleans of small swept solids against closed-form volume and area, each result checked valid
// Adapted from springrep@a03e8a1 tests/booleans.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: crate paths springrep:: -> yack_springrep::; this header added. Moved to crates/yack/tests at W11 (springrep deleted): paths to the crates the items live in.
//! Booleans of small swept solids against closed-form volume and area, each result checked valid
//! and watertight, with face names following history.
// W11: springrep's own kernel tests, moved here when springrep went (ruling (b)). They build with the
// kernel crates directly and compute their expected values, so the facade's maths ban and the
// workspace lints' test-only exemptions (which reach only #[test] fns) do not fit them.
// TODO(api-owner, hardening, D26): rewrite these tests to the lints, test by test, and drop this allow.
#![allow(clippy::disallowed_methods, clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::indexing_slicing)]

use std::f64::consts::{PI, TAU};

use yack_base::kernel::Error;
use yack_boolean::difference;
use yack_boolean::intersection;
use yack_boolean::union;
use yack_check as check;
use yack_math::kernel::Frame;
use yack_math::kernel::Transform;
use yack_math::kernel::Vec2;
use yack_math::kernel::Vec3;
use yack_measure as measure;
use yack_mesh as mesh;
use yack_mesh::MeshOptions;
use yack_profile::NamedSegment;
use yack_profile::PointsInterp;
use yack_profile::Profile;
use yack_profile::ProfileLoop;
use yack_profile::Segment;
use yack_build::sweep::extrude;
use yack_build::sweep::revolve;
use yack_topo::Brep;

const COARSE: MeshOptions = MeshOptions {
    linear: 0.2,
    angular: 0.5,
};

fn block(id: &str, lo: [f64; 3], hi: [f64; 3]) -> Brep {
    let c = Vec2::new(0.5 * (lo[0] + hi[0]), 0.5 * (lo[1] + hi[1]));
    let frame = Frame::new(Vec3::new(0.0, 0.0, lo[2]), Vec3::Z, Vec3::X).unwrap();
    extrude(
        &Profile::rect(c, hi[0] - lo[0], hi[1] - lo[1]),
        &frame,
        hi[2] - lo[2],
        id,
    )
    .unwrap()
}

/// A cylinder of radius `r` along `axis` from `base`, `len` long.
fn rod(id: &str, base: [f64; 3], axis: [f64; 3], r: f64, len: f64) -> Brep {
    let z = Vec3::from_array(axis);
    let frame = Frame::from_axis(Vec3::from_array(base), z).unwrap();
    extrude(&Profile::circle(Vec2::ZERO, 2.0 * r), &frame, len, id).unwrap()
}

fn ball(id: &str, centre: [f64; 3], r: f64) -> Brep {
    let seg = |segment: Segment, name: &str| NamedSegment::new(segment, name);
    let p = Profile {
        loops: vec![ProfileLoop {
            segments: vec![
                seg(
                    Segment::Arc {
                        center: Vec2::ZERO,
                        radius: r,
                        start: -PI / 2.0,
                        sweep: PI,
                    },
                    "surface",
                ),
                seg(
                    Segment::Line {
                        a: Vec2::new(0.0, r),
                        b: Vec2::new(0.0, -r),
                    },
                    "axis",
                ),
            ],
        }],
    };
    let c = Vec3::from_array(centre);
    let frame = Frame::new(c, Vec3::new(0.0, -1.0, 0.0), Vec3::X).unwrap();
    revolve(&p, &frame, c, Vec3::Z, TAU, id).unwrap()
}

fn assert_solid(b: &Brep, volume: f64, area: f64) {
    let issues = check::check(b);
    assert!(issues.is_empty(), "{issues:?}");
    let v = measure::volume(b);
    let a = measure::area(b);
    assert!(
        (v - volume).abs() <= 1e-9 * volume,
        "volume {v} vs {volume}"
    );
    assert!((a - area).abs() <= 1e-9 * area, "area {a} vs {area}");
    let m = mesh::tessellate(b, &COARSE).unwrap();
    for s in b.solid_ids() {
        assert!(mesh::report(b, &m, s).watertight());
    }
}

fn names(b: &Brep, name: &str) -> usize {
    b.name_counts().get(name).copied().unwrap_or(0)
}

#[test]
fn overlapping_blocks_unite_intersect_and_subtract() {
    let a = block("a", [0.0, 0.0, 0.0], [10.0, 10.0, 10.0]);
    let b = block("b", [5.0, 5.0, 5.0], [15.0, 15.0, 15.0]);
    let u = union(&a, &b).unwrap();
    assert_solid(&u, 2000.0 - 125.0, 2.0 * 600.0 - 6.0 * 25.0);
    assert_eq!(u.face_count(), 12);
    let i = intersection(&a, &b).unwrap();
    assert_solid(&i, 125.0, 150.0);
    for n in [
        "a.end", "a.u_max", "a.v_max", "b.start", "b.u_min", "b.v_min",
    ] {
        assert_eq!(names(&i, n), 1, "{n}");
    }
    assert_eq!(names(&i, "a.start") + names(&i, "b.end"), 0);
    let d = difference(&a, &b).unwrap();
    assert_solid(&d, 875.0, 600.0);
    assert_eq!(names(&d, "b.start"), 1);
}

#[test]
fn a_tool_inside_leaves_a_void() {
    let a = block("a", [0.0, 0.0, 0.0], [10.0, 10.0, 10.0]);
    let b = block("b", [3.0, 3.0, 3.0], [6.0, 6.0, 6.0]);
    let d = difference(&a, &b).unwrap();
    assert_eq!(d.solid_ids().count(), 1);
    assert_eq!(d.solid(d.solid_ids().next().unwrap()).shells.len(), 2);
    assert_solid(&d, 1000.0 - 27.0, 600.0 + 54.0);
    let u = union(&a, &b).unwrap();
    assert_solid(&u, 1000.0, 600.0);
    let i = intersection(&a, &b).unwrap();
    assert_solid(&i, 27.0, 54.0);
}

#[test]
fn disjoint_solids_stay_apart() {
    let a = block("a", [0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
    let b = block("b", [3.0, 0.0, 0.0], [4.0, 1.0, 1.0]);
    let u = union(&a, &b).unwrap();
    assert_eq!(u.solid_ids().count(), 2);
    assert_solid(&u, 2.0, 12.0);
    assert_eq!(intersection(&a, &b).unwrap().solid_ids().count(), 0);
    assert_solid(&difference(&a, &b).unwrap(), 1.0, 6.0);
}

#[test]
fn a_rod_through_a_block_is_a_hole() {
    let a = block("a", [-10.0, -10.0, 0.0], [10.0, 10.0, 5.0]);
    let h = rod("h", [2.0, 1.0, -3.0], [0.0, 0.0, 1.0], 3.0, 20.0);
    let d = difference(&a, &h).unwrap();
    assert_solid(
        &d,
        2000.0 - 45.0 * PI,
        2.0 * (400.0 - 9.0 * PI) + 400.0 + 30.0 * PI,
    );
    assert_eq!(names(&d, "h.side"), 1);
    let u = union(&a, &h).unwrap();
    // The rod's caps replace the discs it takes from the block's faces.
    assert_solid(&u, 2000.0 + 15.0 * 9.0 * PI, 1200.0 + 6.0 * PI * 15.0);
    assert_eq!(names(&u, "h.side"), 2);
}

#[test]
fn a_ball_cut_by_a_plane_face_leaves_a_cap() {
    let s = ball("s", [0.0, 0.0, 0.0], 5.0);
    let a = block("a", [-10.0, -10.0, 3.0], [10.0, 10.0, 10.0]);
    let cap = intersection(&s, &a).unwrap();
    let h = 2.0;
    let disc = PI * (25.0 - 9.0);
    assert_solid(
        &cap,
        PI * h * h * (15.0 - h) / 3.0,
        2.0 * PI * 5.0 * h + disc,
    );
}

#[test]
fn coplanar_faces_merge_and_carry_both_names() {
    let a = block("a", [0.0, 0.0, 0.0], [10.0, 10.0, 10.0]);
    let b = block("b", [10.0, 0.0, 0.0], [20.0, 10.0, 10.0]);
    let u = union(&a, &b).unwrap();
    assert_solid(&u, 2000.0, 1000.0);
    assert_eq!(u.face_count(), 6);
    assert_eq!(names(&u, "a.end"), 1);
    let top = u
        .face_ids()
        .find(|&f| u.face(f).names.iter().any(|n| n.to_string() == "a.end"))
        .unwrap();
    assert!(u.face(top).names.iter().any(|n| n.to_string() == "b.end"));
}

/// The volume two cylinders of radii `a ≥ b` share when their axes cross at right angles.
fn crossing_volume(a: f64, b: f64) -> f64 {
    let n = 2000;
    let (h, mut sum) = (2.0 * b / n as f64, 0.0);
    for i in 0..n {
        let t = -b + h * (i as f64 + 0.5);
        sum += 4.0 * ((b * b - t * t) * (a * a - t * t)).sqrt();
    }
    sum * h
}

#[test]
fn equal_crossing_holes_split_each_other_at_the_touching_points() {
    let cube = block("k", [-10.0, -10.0, -10.0], [10.0, 10.0, 10.0]);
    let hx = rod("hx", [-10.0, 0.0, 0.0], [1.0, 0.0, 0.0], 4.0, 1000.0);
    let hy = rod("hy", [0.0, -10.0, 0.0], [0.0, 1.0, 0.0], 4.0, 1000.0);
    let two = difference(&difference(&cube, &hx).unwrap(), &hy).unwrap();
    let steinmetz = 16.0 * 64.0 / 3.0;
    let area = 6.0 * 400.0 - 4.0 * 16.0 * PI + 2.0 * 8.0 * PI * 20.0 - 16.0 * 16.0;
    assert_solid(&two, 8000.0 - 2.0 * 16.0 * PI * 20.0 + steinmetz, area);
    assert_eq!(names(&two, "hx.side"), 2);
    assert_eq!(names(&two, "hy.side"), 2);
}

#[test]
fn holes_a_hair_from_tangent_stay_valid() {
    let a = block("a", [0.0, 0.0, 0.0], [20.0, 20.0, 10.0]);
    for (x, faces) in [(5.000001, 7), (4.999999, 8)] {
        let h = rod("h", [x, 10.0, 0.0], [0.0, 0.0, 1.0], 5.0, 1000.0);
        let d = difference(&a, &h).unwrap();
        assert!(check::check(&d).is_empty(), "{x}");
        assert_eq!(d.face_count(), faces, "{x}");
        let lens = if x < 5.0 {
            let (c, r) = (x, 5.0f64);
            r * r * (c / r).acos() - c * (r * r - c * c).sqrt()
        } else {
            0.0
        };
        let v = 4000.0 - 10.0 * (25.0 * PI - lens);
        assert!((measure::volume(&d) - v).abs() < 1e-9 * v, "{x}");
    }
}

#[test]
fn coaxial_cut_with_coincident_caps_leaves_a_tube() {
    let outer = rod("o", [0.0, 0.0, 0.0], [0.0, 0.0, 1.0], 10.0, 15.0);
    let bore = rod("b", [0.0, 0.0, 0.0], [0.0, 0.0, 1.0], 5.0, 15.0);
    let t = difference(&outer, &bore).unwrap();
    assert_solid(
        &t,
        PI * 75.0 * 15.0,
        2.0 * PI * 75.0 + 2.0 * PI * 15.0 * 15.0,
    );
    assert_eq!(names(&t, "b.start") + names(&t, "b.end"), 0);
    assert_eq!(t.face_count(), 4);
}

#[test]
fn coaxial_rods_unite_into_one_side_across_their_seams() {
    let a = rod("a", [0.0, 0.0, 0.0], [0.0, 0.0, 1.0], 5.0, 10.0);
    let mut b = rod("b", [0.0, 0.0, 5.0], [0.0, 0.0, 1.0], 5.0, 10.0);
    b.transform(&Transform::rotation(Vec3::ZERO, Vec3::Z, 1.0));
    let u = union(&a, &b).unwrap();
    assert_solid(&u, PI * 25.0 * 15.0, 2.0 * PI * 25.0 + 10.0 * PI * 15.0);
    assert_eq!(u.face_count(), 3);
    assert_eq!(names(&u, "a.side") + names(&u, "b.side"), 2);
}

#[test]
fn a_hole_through_a_rod_crosses_its_seam() {
    let body = rod("body", [0.0, 0.0, 0.0], [0.0, 0.0, 1.0], 10.0, 30.0);
    let h = rod("h", [-15.0, 0.0, 15.0], [1.0, 0.0, 0.0], 3.0, 1000.0);
    let d = difference(&body, &h).unwrap();
    let v = PI * 100.0 * 30.0 - crossing_volume(10.0, 3.0);
    assert!(check::check(&d).is_empty());
    assert!((measure::volume(&d) - v).abs() < 1e-6 * v);
    assert_eq!(d.face_count(), 4);
}

#[test]
fn a_plane_through_a_balls_poles_halves_it() {
    let s = ball("s", [0.0, 0.0, 0.0], 5.0);
    let half = intersection(&s, &block("k", [0.0, -10.0, -10.0], [10.0, 10.0, 10.0])).unwrap();
    assert_solid(&half, 2.0 / 3.0 * PI * 125.0, 2.0 * PI * 25.0 + PI * 25.0);
    assert_eq!(half.face_count(), 2);
}

#[test]
fn a_counterbore_and_a_boss_share_caps_with_the_plate() {
    let plate = block("p", [0.0, 0.0, 0.0], [40.0, 40.0, 10.0]);
    let h = rod("h", [20.0, 20.0, 0.0], [0.0, 0.0, 1.0], 5.0, 1000.0);
    let holed = difference(&plate, &h).unwrap();
    let cb = rod("cb", [20.0, 20.0, 10.0], [0.0, 0.0, -1.0], 8.0, 4.0);
    let bored = difference(&holed, &cb).unwrap();
    assert_solid(
        &bored,
        16000.0 - 250.0 * PI - 4.0 * 39.0 * PI,
        2.0 * 1600.0 + 1600.0 - 64.0 * PI - 25.0 * PI
            + 39.0 * PI
            + 16.0 * PI * 4.0
            + 10.0 * PI * 6.0,
    );
    assert_eq!(names(&bored, "cb.end"), 1);
    let boss = rod("b", [20.0, 20.0, 10.0], [0.0, 0.0, 1.0], 5.0, 8.0);
    let u = union(&plate, &boss).unwrap();
    assert_solid(
        &u,
        16000.0 + 200.0 * PI,
        2.0 * 1600.0 + 1600.0 + 10.0 * PI * 8.0,
    );
    assert_eq!(names(&u, "b.start"), 0);
}

/// The circle where two balls meet bounds both sphere faces, whose meshes refine against it.
#[test]
fn overlapping_balls_unite_and_mesh() {
    let (big, small) = (5.0, 4.0);
    let d = 14f64.sqrt();
    let lens = PI
        * (big + small - d).powi(2)
        * (d * d + 2.0 * d * small - 3.0 * small * small + 2.0 * d * big + 6.0 * small * big
            - 3.0 * big * big)
        / (12.0 * d);
    let volume = 4.0 / 3.0 * PI * (big.powi(3) + small.powi(3)) - lens;
    let h1 = (small - big + d) * (small + big - d) / (2.0 * d);
    let h2 = (big - small + d) * (big + small - d) / (2.0 * d);
    let area = 4.0 * PI * (big * big + small * small) - 2.0 * PI * (big * h1 + small * h2);
    let u = union(
        &ball("a", [0.0, 0.0, 0.0], big),
        &ball("b", [3.0, 2.0, 1.0], small),
    )
    .unwrap();
    assert_solid(&u, volume, area);
}

#[test]
fn a_solid_minus_itself_is_empty() {
    let a = block("a", [0.0, 0.0, 0.0], [10.0, 10.0, 10.0]);
    for r in [
        difference(&a, &a.clone()).unwrap(),
        intersection(&a, &block("b", [20.0, 0.0, 0.0], [30.0, 10.0, 10.0])).unwrap(),
    ] {
        assert_eq!(r.solid_ids().count(), 0);
        assert_eq!(r.face_count(), 0);
        assert!(check::check(&r).is_empty());
    }
}

/// Within the tolerance of tangent a hole touches the wall: the block touches itself along that
/// line, one solid round an edge four faces use. Just past it the hole cuts a slot, its cap's
/// sliver outside the wall decided by the exact side.
#[test]
fn holes_at_the_tolerance_from_tangent_touch_or_cut() {
    let a = block("a", [0.0, 0.0, 0.0], [20.0, 20.0, 10.0]);
    for x in [5.0 + 5e-8, 5.0 - 5e-8, 5.0 - 9e-8] {
        let h = rod("h", [x, 10.0, 0.0], [0.0, 0.0, 1.0], 5.0, 1000.0);
        let d = difference(&a, &h).unwrap();
        assert!(check::check(&d).is_empty(), "{x}: {:?}", check::check(&d));
        assert_eq!(d.solid_ids().count(), 1, "{x}");
        assert_eq!(d.radial_edges().count(), 1, "{x}");
        let v = 4000.0 - 250.0 * PI;
        assert!((measure::volume(&d) - v).abs() < 1e-9 * v, "{x}");
    }
    for x in [5.0 - 1.5e-7, 5.0 - 3e-7] {
        let h = rod("h", [x, 10.0, 0.0], [0.0, 0.0, 1.0], 5.0, 1000.0);
        let d = difference(&a, &h).unwrap();
        assert!(check::check(&d).is_empty(), "{x}");
        assert_eq!(d.face_count(), 8, "{x}");
        let (c, r) = (x, 5.0f64);
        let lens = r * r * (c / r).acos() - c * (r * r - c * c).sqrt();
        let v = 4000.0 - 10.0 * (25.0 * PI - lens);
        assert!((measure::volume(&d) - v).abs() < 1e-9 * v, "{x}");
    }
}

/// Faces 1.5 times the tolerance off coplanar leave slivers that are classified by their exact
/// side rather than refused.
#[test]
fn faces_just_past_the_tolerance_from_coplanar_unite() {
    let a = block("a", [0.0, 0.0, 0.0], [20.0, 20.0, 10.0]);
    let b = block("b", [20.0, 0.0, 1.5e-7], [40.0, 20.0, 10.0 + 1.5e-7]);
    let u = union(&a, &b).unwrap();
    assert!(check::check(&u).is_empty());
    assert!((measure::volume(&u) - 8000.0).abs() < 1e-9 * 8000.0);
}

/// A relief bore 0.05 mm over a bore leaves annuli thinner than a polygon of their circles can
/// find an inside point in.
#[test]
fn a_relief_bore_leaves_thin_annuli() {
    let body = rod("b", [0.0, 0.0, 0.0], [0.0, 0.0, 1.0], 52.5, 45.0);
    let bore = rod("h", [0.0, 0.0, -1.0], [0.0, 0.0, 1.0], 45.0, 47.0);
    let relief = rod("r", [0.0, 0.0, 10.0], [0.0, 0.0, 1.0], 45.05, 24.0);
    let d = difference(&difference(&body, &bore).unwrap(), &relief).unwrap();
    let v =
        PI * (52.5f64.powi(2) - 45.0f64.powi(2)) * 45.0 - PI * (45.05f64.powi(2) - 2025.0) * 24.0;
    assert!(check::check(&d).is_empty());
    assert!((measure::volume(&d) - v).abs() < 1e-9 * v);
}

fn spline_disc(z: f64, h: f64) -> Brep {
    let pts: Vec<Vec2> = (0..8)
        .map(|i| {
            let a = i as f64 * TAU / 8.0;
            Vec2::new(6.0 * a.cos(), 4.0 * a.sin())
        })
        .collect();
    let p = Profile::points(&pts, PointsInterp::PeriodicSpline, "s").unwrap();
    extrude(&p, &Frame::WORLD.translated(Vec3::new(0.0, 0.0, z)), h, "e").unwrap()
}

/// A solid with spline edges against an exact copy keeps its edges whole: curves that run together
/// meet only at their ends.
#[test]
fn a_spline_solid_unites_with_itself_unchanged() {
    let a = spline_disc(0.0, 2.0);
    let v = measure::volume(&a);
    for r in [
        union(&a, &a.clone()).unwrap(),
        intersection(&a, &a.clone()).unwrap(),
    ] {
        assert!(check::check(&r).is_empty());
        assert_eq!(r.edge_ids().count(), a.edge_ids().count());
        assert!((measure::volume(&r) - v).abs() < 1e-9 * v);
        let m = mesh::tessellate(&r, &COARSE).unwrap();
        assert!(mesh::report(&r, &m, r.solid_ids().next().unwrap()).watertight());
    }
}

/// Two extrusions of one spline in frames a height apart have the same side surface, so stacked
/// they unite into one side face.
#[test]
fn stacked_spline_extrusions_share_their_side() {
    let (a, b) = (spline_disc(0.0, 2.0), spline_disc(2.0, 3.0));
    let u = union(&a, &b).unwrap();
    assert!(check::check(&u).is_empty());
    assert_eq!(u.face_count(), 3);
    let v = measure::volume(&a) / 2.0 * 5.0;
    assert!((measure::volume(&u) - v).abs() < 1e-9 * v);
}

#[test]
fn a_rod_touching_a_box_face_along_a_line_takes_nothing_from_it() {
    // The rod's side touches the face x = 10 along its middle, where both faces' interior points
    // fall; overlaps within the tolerance are the same touch.
    for overlap in [0.0, 1e-9, 5e-8, 1e-7] {
        let a = block("a", [0.0, 0.0, 0.0], [10.0, 10.0, 10.0]);
        let c = rod("c", [15.0 - overlap, 5.0, 2.0], [0.0, 0.0, 1.0], 5.0, 6.0);
        let d = difference(&a, &c).unwrap();
        assert_solid(&d, 1000.0, 600.0);
        assert_eq!(intersection(&a, &c).unwrap().solid_ids().count(), 0);
    }
}

#[test]
fn a_tube_a_hair_off_the_axis_of_its_pocket_meets_the_wall_in_a_crescent() {
    // A tube of outer radius 2.4 in a pocket of the same radius, its axis 0.38 µm off the
    // pocket's, as an insert placed by a pose given to a tenth of a micron sits in its wall: it
    // meets the wall only in the crescent of its circle outside the pocket's, along its 4 mm. Its
    // end faces are lunes 0.38 µm wide whose chords sag 30 times further, so whether they check
    // valid turns on the angle the tube's circles start at, which each turn below varies.
    let r = 2.4;
    let (ex, ey) = (1.95e-4f64, 3.31e-4f64);
    let e = (ex * ex + ey * ey).sqrt();
    let wall = difference(
        &block("wall", [-6.0, -6.0, 0.0], [6.0, 6.0, 8.0]),
        &rod("pocket", [0.0, 0.0, 2.0], [0.0, 0.0, 1.0], r, 7.0),
    )
    .unwrap();
    let lens = 2.0 * r * r * (e / (2.0 * r)).acos() - 0.5 * e * (4.0 * r * r - e * e).sqrt();
    let want = (PI * r * r - lens) * 4.0;
    for turn in [0.5f64, 2.3, 2.7, 4.0, 5.5] {
        let at = |z: f64| {
            let x = Vec3::new(turn.cos(), turn.sin(), 0.0);
            Frame::new(Vec3::new(ex, ey, z), Vec3::Z, x).unwrap()
        };
        let tube = difference(
            &extrude(&Profile::circle(Vec2::ZERO, 2.0 * r), &at(2.4), 4.0, "o").unwrap(),
            &extrude(&Profile::circle(Vec2::ZERO, 3.0), &at(1.4), 6.0, "b").unwrap(),
        )
        .unwrap();
        let x = intersection(&wall, &tube).unwrap_or_else(|e| panic!("turn {turn}: {e}"));
        assert!(check::check(&x).is_empty(), "turn {turn}");
        let got = measure::volume(&x);
        assert!(
            (got - want).abs() <= 1e-7 * want,
            "turn {turn}: {got} vs {want}"
        );
    }
}

#[test]
fn faces_within_a_micron_of_each_other_all_over_only_touch() {
    // A is the cube 0..4; B starts `o` inside A's face x = 4, its faces y = 0 and y = 4 on A's,
    // from below A's floor to z = 3. B's face lies within o of A's all over, so where o is under
    // the micron that a part placed by a rounded pose sits off its seat, the two only touch: no
    // common part, A's 64 and B's 64 together, A's 64 alone. Past it, the sliver o × 4 × 3 is kept.
    let a = block("a", [0.0; 3], [4.0; 3]);
    for o in [1.2e-7, 1.6e-7, 5e-7, 9e-7, 3e-6] {
        let b = block("b", [4.0 - o, 0.0, -1.0], [8.0, 4.0, 3.0]);
        let common = if o < 1e-6 { 0.0 } else { o * 4.0 * 3.0 };
        let b_volume = if o < 1e-6 { 64.0 } else { (4.0 + o) * 16.0 };
        for (name, r, want) in [
            ("common", intersection(&a, &b), common),
            ("union", union(&a, &b), 64.0 + b_volume - common),
            ("cut", difference(&a, &b), 64.0 - common),
        ] {
            let x = r.unwrap_or_else(|e| panic!("{o} {name}: {e}"));
            assert!(check::check(&x).is_empty(), "{o} {name}");
            let got = measure::volume(&x);
            assert!((got - want).abs() <= 1e-11, "{o} {name}: {got} vs {want}");
        }
    }
}

#[test]
fn a_tube_within_a_micron_of_its_pockets_axis_only_touches_the_wall() {
    // As the crescent test's tube, its axis 0.51 µm off the pocket's: the crescent's tips would run
    // within the tolerance of both cylinders for half a millimetre, so the tube sits in the pocket.
    let r = 2.4;
    let wall = difference(
        &block("wall", [-6.0, -6.0, 0.0], [6.0, 6.0, 8.0]),
        &rod("pocket", [0.0, 0.0, 2.0], [0.0, 0.0, 1.0], r, 7.0),
    )
    .unwrap();
    for turn in [0.5f64, 2.7] {
        let x = Vec3::new(turn.cos(), turn.sin(), 0.0);
        let at = |z: f64| Frame::new(Vec3::new(3e-7, 4.1e-7, z), Vec3::Z, x).unwrap();
        let tube = difference(
            &extrude(&Profile::circle(Vec2::ZERO, 2.0 * r), &at(2.4), 4.0, "o").unwrap(),
            &extrude(&Profile::circle(Vec2::ZERO, 3.0), &at(1.4), 6.0, "b").unwrap(),
        )
        .unwrap();
        let common = intersection(&wall, &tube).unwrap_or_else(|e| panic!("turn {turn}: {e}"));
        assert_eq!(common.solid_ids().count(), 0, "turn {turn}");
    }
}

#[test]
fn flush_faces_crossing_at_a_hair_meet_along_their_line() {
    // A 2 × 2 × 2 box in the cube 0..4, its face x = 4 on the cube's, turned 1.56e-5 rad about an
    // oblique axis through that face and moved by up to 1e-5 mm: the two faces cross along a line
    // across the box's face, as a plate placed by a pose rounded in its fifth decimal meets its
    // cover. Planes that far off parallel put their meeting line 1e-5 mm off both where its point
    // was found as the difference of two numbers 1 − c² apart. Every result is valid, and the
    // three agree: common and union add up to both, and the cube less the box is the cube less
    // the common part, a hair under the box's 8 where a wedge of it runs past the cube's face.
    let a = block("a", [0.0; 3], [4.0; 3]);
    for (turn, off) in [(1.56e-5f64, 0.0f64), (1.56e-5, -1e-5), (2.4e-5, 0.0)] {
        let mut b = block("b", [2.0, 1.0, 1.0], [4.0, 3.0, 3.0]);
        let axis = Vec3::new(0.0, 0.6, 0.8);
        b.transform(&Transform::rotation(Vec3::new(4.0, 2.2, 1.9), axis, turn));
        b.transform(&Transform::translation(Vec3::new(off, 0.0, 0.0)));
        let run = |r: Result<Brep, Error>, name: &str| {
            let x = r.unwrap_or_else(|e| panic!("{turn} {off} {name}: {e}"));
            assert!(check::check(&x).is_empty(), "{turn} {off} {name}");
            measure::volume(&x)
        };
        let common = run(intersection(&a, &b), "common");
        let joined = run(union(&a, &b), "union");
        let cut = run(difference(&a, &b), "cut");
        let vb = measure::volume(&b);
        assert!((common + joined - 64.0 - vb).abs() <= 1e-11, "{turn} {off}");
        assert!((cut - (64.0 - common)).abs() <= 1e-11, "{turn} {off}");
        assert!(
            common < 8.0 && common > 8.0 - 1e-3,
            "{turn} {off}: {common}"
        );
    }
}

/// Two cylinders micrometres across crossing at an odd angle: a loop's parameter images meet to a
/// gap far below the tolerance in space, however large it is in radians on so small a radius.
/// Valid every way round, the four results agreeing: |a ∪ b| = |a| + |b| − |a ∩ b| and so on.
#[test]
fn cylinders_micrometres_across_crossing_aslant_combine() {
    let cyl = |o: [f64; 3], z: [f64; 3], x: [f64; 3], r: f64, h: f64| {
        let f = Frame::new(
            Vec3::from_array(o),
            Vec3::from_array(z),
            Vec3::from_array(x),
        )
        .unwrap();
        extrude(&Profile::circle(Vec2::ZERO, 2.0 * r), &f, h, "c").unwrap()
    };
    let a = cyl(
        [0.0, 0.0, -0.015],
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 0.0],
        0.007173807691792167,
        0.03,
    );
    let b = cyl(
        [
            -0.002532118538179013,
            0.0072643840011615635,
            0.008785851568869818,
        ],
        [0.2734748768191249, -0.9104945125904419, -0.3101793582614065],
        [0.9618791461242749, 0.25886555049919935, 0.08818806617667722],
        0.0053024717958101335,
        0.03,
    );
    let results = [
        union(&a, &b),
        intersection(&a, &b),
        difference(&a, &b),
        difference(&b, &a),
    ];
    let mut vols = Vec::new();
    for r in results {
        let r = r.unwrap();
        assert!(check::check(&r).is_empty(), "{:?}", check::check(&r));
        vols.push(measure::volume(&r));
    }
    // Exact to the tolerance in space, so to it times the area in volume.
    let (va, vb) = (measure::volume(&a), measure::volume(&b));
    let bound = 1e-7 * (measure::area(&a) + measure::area(&b));
    assert!((vols[0] - (va + vb - vols[1])).abs() <= bound);
    assert!((vols[2] - (va - vols[1])).abs() <= bound);
    assert!((vols[3] - (vb - vols[1])).abs() <= bound);
}

/// A round thread, a 0.37 mm circle 8.01 mm out, screwed half a turn and a little, cut by a rod on
/// its axis whose side bites through the thread's section: the intersection is the same solid
/// whichever operand comes first, and valid.
#[test]
fn a_thread_and_a_rod_biting_into_it_intersect_either_way_round() {
    use yack_build::sweep::Helix;
    use yack_build::sweep::helix;
    let profile = Profile::circle(
        Vec2::new(8.013653159337178, 0.5917483297087343),
        2.0 * 0.37382307626072764,
    );
    let meridian = Frame::new(Vec3::ZERO, Vec3::new(0.0, -1.0, 0.0), Vec3::X).unwrap();
    let motion = Helix {
        origin: Vec3::ZERO,
        axis: Vec3::Z,
        rise: 0.25338470539470814,
        sign: 1.0,
    };
    let thread = helix(&profile, &meridian, &motion, 3.1916513479467015, "hel").unwrap();
    let rod = rod(
        "r",
        [0.0, 0.0, -1.0],
        [0.0, 0.0, 1.0],
        7.932446399148578,
        3.9922122959395665,
    );
    let ab = intersection(&thread, &rod).unwrap();
    let ba = intersection(&rod, &thread).unwrap();
    for x in [&ab, &ba] {
        assert!(check::check(x).is_empty(), "{:?}", check::check(x));
    }
    // Exact to the tolerance in space, so to it times the area in volume.
    let (va, vb) = (measure::volume(&ab), measure::volume(&ba));
    assert!((va - vb).abs() <= 1e-7 * measure::area(&ab), "{va} vs {vb}");
}
