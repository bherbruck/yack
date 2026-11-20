// yack: booleans of small solids against closed-form volume and area, each result checked valid and watertight.
// Adapted from springrep@a03e8a1 tests/booleans.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: springrep API replaced by the yack public API (PLAN §4.3); springrep `Frame::new(o, z, x)` and
// `Frame::from_axis` written as yack `Frame::new(o, x, y, z)` with the same axes; `Profile::circle` takes a
// radius (springrep took a diameter); solids moved by `Transform` are built in frames moved by the same
// `Rigid`; a solid count is `betti()[0]` and a void shell `betti()[2]`; springrep's face names "f.part" are
// the faces tagged `f` (`Solid::named`) picked out by the part's geometry (its cap plane or cylindrical side).

use std::f64::consts::{PI, TAU};

use yack::prelude::*;
use yack::{EdgeId, FaceRef};

/// The original's `MeshOptions { linear: 0.2, .. }`.
const COARSE: f64 = 0.2;

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

/// springrep's `Frame::from_axis(base, z)`: x is z × (X, or Y when z is near X), y is z × x.
fn axis_frame(base: [f64; 3], z: [f64; 3]) -> Frame {
    let hint = if z[0].abs() < 0.6 { [1.0, 0.0, 0.0] } else { [0.0, 1.0, 0.0] };
    let x = cross(z, hint);
    let n = (x[0] * x[0] + x[1] * x[1] + x[2] * x[2]).sqrt();
    let x = [x[0] / n, x[1] / n, x[2] / n];
    Frame::new(base, x, cross(z, x), z).unwrap()
}

/// The box `lo`–`hi`.
fn bx(lo: [f64; 3], hi: [f64; 3]) -> Solid {
    block(&Frame::at(lo), [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]]).unwrap()
}

/// A cylinder of radius `r` along `axis` from `base`, `len` long.
fn rod(base: [f64; 3], axis: [f64; 3], r: f64, len: f64) -> Solid {
    cylinder(&axis_frame(base, axis), r, len).unwrap()
}

fn named_bx(tag: &str, lo: [f64; 3], hi: [f64; 3]) -> Solid {
    bx(lo, hi).named(tag)
}

fn named_rod(tag: &str, base: [f64; 3], axis: [f64; 3], r: f64, len: f64) -> Solid {
    rod(base, axis, r, len).named(tag)
}

fn ball(centre: [f64; 3], r: f64) -> Solid {
    sphere(&Frame::at(centre), r).unwrap()
}

fn assert_solid(b: &Solid, volume: f64, area: f64) {
    let issues = b.check();
    assert!(issues.is_empty(), "{issues:?}");
    let v = b.volume();
    let a = b.area();
    assert!((v - volume).abs() <= 1e-9 * volume, "volume {v} vs {volume}");
    assert!((a - area).abs() <= 1e-9 * area, "area {a} vs {area}");
    let m = b.mesh(COARSE).unwrap();
    // API GAP: springrep reports watertightness per solid of the set; yack's Mesh is one mesh of the
    // whole set, so it is checked as one.
    assert!(m.is_watertight());
}

/// The edges four faces use: where pieces meet along a curve (springrep's radial edges).
fn four_face_edges(b: &Solid) -> Vec<EdgeId> {
    b.edges().into_iter().filter(|&e| b.edge(e).unwrap().uses() == 4).collect()
}

/// A face's plane is `coordinate[axis] = at` (springrep's cap names: a block's start/end/u_*/v_*, a
/// rod's start/end).
fn plane_at(f: &FaceRef, axis: usize, at: f64) -> bool {
    match f.kind() {
        SurfaceKind::Plane { normal } => {
            let (n, c) = ([normal.x, normal.y, normal.z], f.centroid());
            n[axis].abs() > 1.0 - 1e-9 && ([c.x, c.y, c.z][axis] - at).abs() < 1e-6
        }
        _ => false,
    }
}

/// A rod's "side": its cylindrical face.
fn side(f: &FaceRef) -> bool {
    matches!(f.kind(), SurfaceKind::Cylinder { .. })
}

/// springrep's `names(b, "tag.part")`: how many faces carry that name.
fn names(b: &Solid, tag: &str, part: impl Fn(&FaceRef) -> bool) -> usize {
    b.faces_where(|f| f.tags().iter().any(|t| t.as_str() == tag) && part(f)).len()
}

#[test]
fn overlapping_blocks_unite_intersect_and_subtract() {
    let a = named_bx("a", [0.0, 0.0, 0.0], [10.0, 10.0, 10.0]);
    let b = named_bx("b", [5.0, 5.0, 5.0], [15.0, 15.0, 15.0]);
    let u = a.union(&b).unwrap();
    assert_solid(&u, 2000.0 - 125.0, 2.0 * 600.0 - 6.0 * 25.0);
    assert_eq!(u.faces().len(), 12);
    let i = a.intersection(&b).unwrap();
    assert_solid(&i, 125.0, 150.0);
    // "a.end", "a.u_max", "a.v_max", "b.start", "b.u_min", "b.v_min".
    for (n, tag, axis, at) in [
        ("a.end", "a", 2, 10.0),
        ("a.u_max", "a", 0, 10.0),
        ("a.v_max", "a", 1, 10.0),
        ("b.start", "b", 2, 5.0),
        ("b.u_min", "b", 0, 5.0),
        ("b.v_min", "b", 1, 5.0),
    ] {
        assert_eq!(names(&i, tag, |f| plane_at(f, axis, at)), 1, "{n}");
    }
    // "a.start" + "b.end".
    assert_eq!(names(&i, "a", |f| plane_at(f, 2, 0.0)) + names(&i, "b", |f| plane_at(f, 2, 15.0)), 0);
    let d = a.difference(&b).unwrap();
    assert_solid(&d, 875.0, 600.0);
    // "b.start".
    assert_eq!(names(&d, "b", |f| plane_at(f, 2, 5.0)), 1);
}

#[test]
fn a_tool_inside_leaves_a_void() {
    let a = bx([0.0, 0.0, 0.0], [10.0, 10.0, 10.0]);
    let b = bx([3.0, 3.0, 3.0], [6.0, 6.0, 6.0]);
    let d = a.difference(&b).unwrap();
    // One solid of two shells: one piece, one void.
    assert_eq!(d.betti()[0], 1);
    assert_eq!(d.betti()[2], 1);
    assert_solid(&d, 1000.0 - 27.0, 600.0 + 54.0);
    let u = a.union(&b).unwrap();
    assert_solid(&u, 1000.0, 600.0);
    let i = a.intersection(&b).unwrap();
    assert_solid(&i, 27.0, 54.0);
}

#[test]
fn disjoint_solids_stay_apart() {
    let a = bx([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
    let b = bx([3.0, 0.0, 0.0], [4.0, 1.0, 1.0]);
    let u = a.union(&b).unwrap();
    assert_eq!(u.betti()[0], 2);
    assert_solid(&u, 2.0, 12.0);
    assert!(a.intersection(&b).unwrap().is_empty());
    assert_solid(&a.difference(&b).unwrap(), 1.0, 6.0);
}

#[test]
fn a_rod_through_a_block_is_a_hole() {
    let a = bx([-10.0, -10.0, 0.0], [10.0, 10.0, 5.0]);
    let h = named_rod("h", [2.0, 1.0, -3.0], [0.0, 0.0, 1.0], 3.0, 20.0);
    let d = a.difference(&h).unwrap();
    assert_solid(&d, 2000.0 - 45.0 * PI, 2.0 * (400.0 - 9.0 * PI) + 400.0 + 30.0 * PI);
    assert_eq!(names(&d, "h", side), 1);
    let u = a.union(&h).unwrap();
    // The rod's caps replace the discs it takes from the block's faces.
    assert_solid(&u, 2000.0 + 15.0 * 9.0 * PI, 1200.0 + 6.0 * PI * 15.0);
    assert_eq!(names(&u, "h", side), 2);
}

#[test]
fn a_ball_cut_by_a_plane_face_leaves_a_cap() {
    let s = ball([0.0, 0.0, 0.0], 5.0);
    let a = bx([-10.0, -10.0, 3.0], [10.0, 10.0, 10.0]);
    let cap = s.intersection(&a).unwrap();
    let h = 2.0;
    let disc = PI * (25.0 - 9.0);
    assert_solid(&cap, PI * h * h * (15.0 - h) / 3.0, 2.0 * PI * 5.0 * h + disc);
}

#[test]
fn coplanar_faces_merge_and_carry_both_names() {
    let a = named_bx("a", [0.0, 0.0, 0.0], [10.0, 10.0, 10.0]);
    let b = named_bx("b", [10.0, 0.0, 0.0], [20.0, 10.0, 10.0]);
    let u = a.union(&b).unwrap();
    assert_solid(&u, 2000.0, 1000.0);
    assert_eq!(u.faces().len(), 6);
    // "a.end" once, and that face also carries "b.end".
    assert_eq!(names(&u, "a", |f| plane_at(f, 2, 10.0)), 1);
    let top = u.faces_where(|f| f.tags().iter().any(|t| t.as_str() == "a") && plane_at(f, 2, 10.0))[0];
    assert!(u.face(top).unwrap().tags().iter().any(|t| t.as_str() == "b"));
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
    let cube = bx([-10.0, -10.0, -10.0], [10.0, 10.0, 10.0]);
    let hx = named_rod("hx", [-10.0, 0.0, 0.0], [1.0, 0.0, 0.0], 4.0, 1000.0);
    let hy = named_rod("hy", [0.0, -10.0, 0.0], [0.0, 1.0, 0.0], 4.0, 1000.0);
    let two = cube.difference(&hx).unwrap().difference(&hy).unwrap();
    let steinmetz = 16.0 * 64.0 / 3.0;
    let area = 6.0 * 400.0 - 4.0 * 16.0 * PI + 2.0 * 8.0 * PI * 20.0 - 16.0 * 16.0;
    assert_solid(&two, 8000.0 - 2.0 * 16.0 * PI * 20.0 + steinmetz, area);
    assert_eq!(names(&two, "hx", side), 2);
    assert_eq!(names(&two, "hy", side), 2);
}

#[test]
fn holes_a_hair_from_tangent_stay_valid() {
    let a = bx([0.0, 0.0, 0.0], [20.0, 20.0, 10.0]);
    for (x, faces) in [(5.000001, 7), (4.999999, 8)] {
        let h = rod([x, 10.0, 0.0], [0.0, 0.0, 1.0], 5.0, 1000.0);
        let d = a.difference(&h).unwrap();
        assert!(d.check().is_empty(), "{x}");
        assert_eq!(d.faces().len(), faces, "{x}");
        let lens = if x < 5.0 {
            let (c, r) = (x, 5.0f64);
            r * r * (c / r).acos() - c * (r * r - c * c).sqrt()
        } else {
            0.0
        };
        let v = 4000.0 - 10.0 * (25.0 * PI - lens);
        assert!((d.volume() - v).abs() < 1e-9 * v, "{x}");
    }
}

#[test]
fn coaxial_cut_with_coincident_caps_leaves_a_tube() {
    let outer = rod([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], 10.0, 15.0);
    let bore = named_rod("b", [0.0, 0.0, 0.0], [0.0, 0.0, 1.0], 5.0, 15.0);
    let t = outer.difference(&bore).unwrap();
    assert_solid(&t, PI * 75.0 * 15.0, 2.0 * PI * 75.0 + 2.0 * PI * 15.0 * 15.0);
    // "b.start" + "b.end".
    assert_eq!(names(&t, "b", |f| plane_at(f, 2, 0.0)) + names(&t, "b", |f| plane_at(f, 2, 15.0)), 0);
    assert_eq!(t.faces().len(), 4);
}

#[test]
fn coaxial_rods_unite_into_one_side_across_their_seams() {
    let a = named_rod("a", [0.0, 0.0, 0.0], [0.0, 0.0, 1.0], 5.0, 10.0);
    // springrep turns the built rod 1 rad about z; here the rod is built in its frame so turned.
    let turn = Rigid::rotation([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], rad(1.0)).unwrap();
    let b = cylinder(&axis_frame([0.0, 0.0, 5.0], [0.0, 0.0, 1.0]).moved(&turn), 5.0, 10.0).unwrap().named("b");
    let u = a.union(&b).unwrap();
    assert_solid(&u, PI * 25.0 * 15.0, 2.0 * PI * 25.0 + 10.0 * PI * 15.0);
    assert_eq!(u.faces().len(), 3);
    assert_eq!(names(&u, "a", side) + names(&u, "b", side), 2);
}

#[test]
fn a_hole_through_a_rod_crosses_its_seam() {
    let body = rod([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], 10.0, 30.0);
    let h = rod([-15.0, 0.0, 15.0], [1.0, 0.0, 0.0], 3.0, 1000.0);
    let d = body.difference(&h).unwrap();
    let v = PI * 100.0 * 30.0 - crossing_volume(10.0, 3.0);
    assert!(d.check().is_empty());
    assert!((d.volume() - v).abs() < 1e-6 * v);
    assert_eq!(d.faces().len(), 4);
}

#[test]
fn a_plane_through_a_balls_poles_halves_it() {
    let s = ball([0.0, 0.0, 0.0], 5.0);
    let half = s.intersection(&bx([0.0, -10.0, -10.0], [10.0, 10.0, 10.0])).unwrap();
    assert_solid(&half, 2.0 / 3.0 * PI * 125.0, 2.0 * PI * 25.0 + PI * 25.0);
    assert_eq!(half.faces().len(), 2);
}

#[test]
fn a_counterbore_and_a_boss_share_caps_with_the_plate() {
    let plate = bx([0.0, 0.0, 0.0], [40.0, 40.0, 10.0]);
    let h = rod([20.0, 20.0, 0.0], [0.0, 0.0, 1.0], 5.0, 1000.0);
    let holed = plate.difference(&h).unwrap();
    let cb = named_rod("cb", [20.0, 20.0, 10.0], [0.0, 0.0, -1.0], 8.0, 4.0);
    let bored = holed.difference(&cb).unwrap();
    assert_solid(
        &bored,
        16000.0 - 250.0 * PI - 4.0 * 39.0 * PI,
        2.0 * 1600.0 + 1600.0 - 64.0 * PI - 25.0 * PI + 39.0 * PI + 16.0 * PI * 4.0 + 10.0 * PI * 6.0,
    );
    // "cb.end": the rod's far cap, at z = 10 − 4.
    assert_eq!(names(&bored, "cb", |f| plane_at(f, 2, 6.0)), 1);
    let boss = named_rod("b", [20.0, 20.0, 10.0], [0.0, 0.0, 1.0], 5.0, 8.0);
    let u = plate.union(&boss).unwrap();
    assert_solid(&u, 16000.0 + 200.0 * PI, 2.0 * 1600.0 + 1600.0 + 10.0 * PI * 8.0);
    // "b.start": the boss's base cap, at z = 10.
    assert_eq!(names(&u, "b", |f| plane_at(f, 2, 10.0)), 0);
}

/// The circle where two balls meet bounds both sphere faces, whose meshes refine against it.
#[test]
fn overlapping_balls_unite_and_mesh() {
    let (big, small) = (5.0, 4.0);
    let d = 14f64.sqrt();
    let lens = PI
        * (big + small - d).powi(2)
        * (d * d + 2.0 * d * small - 3.0 * small * small + 2.0 * d * big + 6.0 * small * big - 3.0 * big * big)
        / (12.0 * d);
    let volume = 4.0 / 3.0 * PI * (big.powi(3) + small.powi(3)) - lens;
    let h1 = (small - big + d) * (small + big - d) / (2.0 * d);
    let h2 = (big - small + d) * (big + small - d) / (2.0 * d);
    let area = 4.0 * PI * (big * big + small * small) - 2.0 * PI * (big * h1 + small * h2);
    let u = ball([0.0, 0.0, 0.0], big).union(&ball([3.0, 2.0, 1.0], small)).unwrap();
    assert_solid(&u, volume, area);
}

#[test]
fn a_solid_minus_itself_is_empty() {
    let a = bx([0.0, 0.0, 0.0], [10.0, 10.0, 10.0]);
    for r in [
        a.difference(&a.clone()).unwrap(),
        a.intersection(&bx([20.0, 0.0, 0.0], [30.0, 10.0, 10.0])).unwrap(),
    ] {
        assert!(r.is_empty());
        assert_eq!(r.faces().len(), 0);
        assert!(r.check().is_empty());
    }
}

/// Within the tolerance of tangent a hole touches the wall: the block touches itself along that
/// line, one solid round an edge four faces use. Just past it the hole cuts a slot, its cap's
/// sliver outside the wall decided by the exact side.
#[test]
fn holes_at_the_tolerance_from_tangent_touch_or_cut() {
    let a = bx([0.0, 0.0, 0.0], [20.0, 20.0, 10.0]);
    for x in [5.0 + 5e-8, 5.0 - 5e-8, 5.0 - 9e-8] {
        let h = rod([x, 10.0, 0.0], [0.0, 0.0, 1.0], 5.0, 1000.0);
        let d = a.difference(&h).unwrap();
        assert!(d.check().is_empty(), "{x}: {:?}", d.check());
        assert_eq!(d.betti()[0], 1, "{x}");
        assert_eq!(four_face_edges(&d).len(), 1, "{x}");
        let v = 4000.0 - 250.0 * PI;
        assert!((d.volume() - v).abs() < 1e-9 * v, "{x}");
    }
    for x in [5.0 - 1.5e-7, 5.0 - 3e-7] {
        let h = rod([x, 10.0, 0.0], [0.0, 0.0, 1.0], 5.0, 1000.0);
        let d = a.difference(&h).unwrap();
        assert!(d.check().is_empty(), "{x}");
        assert_eq!(d.faces().len(), 8, "{x}");
        let (c, r) = (x, 5.0f64);
        let lens = r * r * (c / r).acos() - c * (r * r - c * c).sqrt();
        let v = 4000.0 - 10.0 * (25.0 * PI - lens);
        assert!((d.volume() - v).abs() < 1e-9 * v, "{x}");
    }
}

/// Faces 1.5 times the tolerance off coplanar leave slivers that are classified by their exact
/// side rather than refused.
#[test]
fn faces_just_past_the_tolerance_from_coplanar_unite() {
    let a = bx([0.0, 0.0, 0.0], [20.0, 20.0, 10.0]);
    let b = bx([20.0, 0.0, 1.5e-7], [40.0, 20.0, 10.0 + 1.5e-7]);
    let u = a.union(&b).unwrap();
    assert!(u.check().is_empty());
    assert!((u.volume() - 8000.0).abs() < 1e-9 * 8000.0);
}

/// A relief bore 0.05 mm over a bore leaves annuli thinner than a polygon of their circles can
/// find an inside point in.
#[test]
fn a_relief_bore_leaves_thin_annuli() {
    let body = rod([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], 52.5, 45.0);
    let bore = rod([0.0, 0.0, -1.0], [0.0, 0.0, 1.0], 45.0, 47.0);
    let relief = rod([0.0, 0.0, 10.0], [0.0, 0.0, 1.0], 45.05, 24.0);
    let d = body.difference(&bore).unwrap().difference(&relief).unwrap();
    let v = PI * (52.5f64.powi(2) - 45.0f64.powi(2)) * 45.0 - PI * (45.05f64.powi(2) - 2025.0) * 24.0;
    assert!(d.check().is_empty());
    assert!((d.volume() - v).abs() < 1e-9 * v);
}

fn spline_disc(z: f64, h: f64) -> Solid {
    let pts: Vec<[f64; 2]> = (0..8)
        .map(|i| {
            let a = i as f64 * TAU / 8.0;
            [6.0 * a.cos(), 4.0 * a.sin()]
        })
        .collect();
    let p = Profile::new(Loop::through(&pts).unwrap(), Vec::new()).unwrap();
    extrude(&Frame::at([0.0, 0.0, z]), &p, h).unwrap()
}

/// A solid with spline edges against an exact copy keeps its edges whole: curves that run together
/// meet only at their ends.
#[test]
fn a_spline_solid_unites_with_itself_unchanged() {
    let a = spline_disc(0.0, 2.0);
    let v = a.volume();
    for r in [a.union(&a.clone()).unwrap(), a.intersection(&a.clone()).unwrap()] {
        assert!(r.check().is_empty());
        assert_eq!(r.edges().len(), a.edges().len());
        assert!((r.volume() - v).abs() < 1e-9 * v);
        let m = r.mesh(COARSE).unwrap();
        assert!(m.is_watertight());
    }
}

/// Two extrusions of one spline in frames a height apart have the same side surface, so stacked
/// they unite into one side face.
#[test]
fn stacked_spline_extrusions_share_their_side() {
    let (a, b) = (spline_disc(0.0, 2.0), spline_disc(2.0, 3.0));
    let u = a.union(&b).unwrap();
    assert!(u.check().is_empty());
    assert_eq!(u.faces().len(), 3);
    let v = a.volume() / 2.0 * 5.0;
    assert!((u.volume() - v).abs() < 1e-9 * v);
}

#[test]
fn a_rod_touching_a_box_face_along_a_line_takes_nothing_from_it() {
    // The rod's side touches the face x = 10 along its middle, where both faces' interior points
    // fall; overlaps within the tolerance are the same touch.
    for overlap in [0.0, 1e-9, 5e-8, 1e-7] {
        let a = bx([0.0, 0.0, 0.0], [10.0, 10.0, 10.0]);
        let c = rod([15.0 - overlap, 5.0, 2.0], [0.0, 0.0, 1.0], 5.0, 6.0);
        let d = a.difference(&c).unwrap();
        assert_solid(&d, 1000.0, 600.0);
        assert!(a.intersection(&c).unwrap().is_empty());
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
    let wall = bx([-6.0, -6.0, 0.0], [6.0, 6.0, 8.0]).difference(&rod([0.0, 0.0, 2.0], [0.0, 0.0, 1.0], r, 7.0)).unwrap();
    let lens = 2.0 * r * r * (e / (2.0 * r)).acos() - 0.5 * e * (4.0 * r * r - e * e).sqrt();
    let want = (PI * r * r - lens) * 4.0;
    for turn in [0.5f64, 2.3, 2.7, 4.0, 5.5] {
        // springrep Frame::new(o, Z, x): x = (cos, sin, 0), y = Z × x.
        let at = |z: f64| {
            let (s, c) = turn.sin_cos();
            Frame::new([ex, ey, z], [c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]).unwrap()
        };
        let tube = extrude(&at(2.4), &Profile::circle([0.0, 0.0], r).unwrap(), 4.0)
            .unwrap()
            .difference(&extrude(&at(1.4), &Profile::circle([0.0, 0.0], 1.5).unwrap(), 6.0).unwrap())
            .unwrap();
        let x = wall.intersection(&tube).unwrap_or_else(|e| panic!("turn {turn}: {e}"));
        assert!(x.check().is_empty(), "turn {turn}");
        let got = x.volume();
        assert!((got - want).abs() <= 1e-7 * want, "turn {turn}: {got} vs {want}");
    }
}

#[test]
fn faces_within_a_micron_of_each_other_all_over_only_touch() {
    // A is the cube 0..4; B starts `o` inside A's face x = 4, its faces y = 0 and y = 4 on A's,
    // from below A's floor to z = 3. B's face lies within o of A's all over, so where o is under
    // the micron that a part placed by a rounded pose sits off its seat, the two only touch: no
    // common part, A's 64 and B's 64 together, A's 64 alone. Past it, the sliver o × 4 × 3 is kept.
    let a = bx([0.0; 3], [4.0; 3]);
    for o in [1.2e-7, 1.6e-7, 5e-7, 9e-7, 3e-6] {
        let b = bx([4.0 - o, 0.0, -1.0], [8.0, 4.0, 3.0]);
        let common = if o < 1e-6 { 0.0 } else { o * 4.0 * 3.0 };
        let b_volume = if o < 1e-6 { 64.0 } else { (4.0 + o) * 16.0 };
        for (name, r, want) in [
            ("common", a.intersection(&b), common),
            ("union", a.union(&b), 64.0 + b_volume - common),
            ("cut", a.difference(&b), 64.0 - common),
        ] {
            let x = r.unwrap_or_else(|e| panic!("{o} {name}: {e}"));
            assert!(x.check().is_empty(), "{o} {name}");
            let got = x.volume();
            assert!((got - want).abs() <= 1e-11, "{o} {name}: {got} vs {want}");
        }
    }
}

#[test]
fn a_tube_within_a_micron_of_its_pockets_axis_only_touches_the_wall() {
    // As the crescent test's tube, its axis 0.51 µm off the pocket's: the crescent's tips would run
    // within the tolerance of both cylinders for half a millimetre, so the tube sits in the pocket.
    let r = 2.4;
    let wall = bx([-6.0, -6.0, 0.0], [6.0, 6.0, 8.0]).difference(&rod([0.0, 0.0, 2.0], [0.0, 0.0, 1.0], r, 7.0)).unwrap();
    for turn in [0.5f64, 2.7] {
        let (s, c) = turn.sin_cos();
        let at = |z: f64| Frame::new([3e-7, 4.1e-7, z], [c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]).unwrap();
        let tube = extrude(&at(2.4), &Profile::circle([0.0, 0.0], r).unwrap(), 4.0)
            .unwrap()
            .difference(&extrude(&at(1.4), &Profile::circle([0.0, 0.0], 1.5).unwrap(), 6.0).unwrap())
            .unwrap();
        let common = wall.intersection(&tube).unwrap_or_else(|e| panic!("turn {turn}: {e}"));
        assert!(common.is_empty(), "turn {turn}");
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
    let a = bx([0.0; 3], [4.0; 3]);
    for (turn, off) in [(1.56e-5f64, 0.0f64), (1.56e-5, -1e-5), (2.4e-5, 0.0)] {
        // springrep transforms the box (2, 1, 1)–(4, 3, 3); here it is built in its frame so moved.
        let rot = Rigid::rotation([4.0, 2.2, 1.9], [0.0, 0.6, 0.8], rad(turn)).unwrap();
        let frame = Frame::at([2.0, 1.0, 1.0]).moved(&rot).moved(&Rigid::translation([off, 0.0, 0.0]));
        let b = block(&frame, [2.0, 2.0, 2.0]).unwrap();
        let run = |r: Result<Solid>, name: &str| {
            let x = r.unwrap_or_else(|e| panic!("{turn} {off} {name}: {e}"));
            assert!(x.check().is_empty(), "{turn} {off} {name}");
            x.volume()
        };
        let common = run(a.intersection(&b), "common");
        let joined = run(a.union(&b), "union");
        let cut = run(a.difference(&b), "cut");
        let vb = b.volume();
        assert!((common + joined - 64.0 - vb).abs() <= 1e-11, "{turn} {off}");
        assert!((cut - (64.0 - common)).abs() <= 1e-11, "{turn} {off}");
        assert!(common < 8.0 && common > 8.0 - 1e-3, "{turn} {off}: {common}");
    }
}

/// Two cylinders micrometres across crossing at an odd angle: a loop's parameter images meet to a
/// gap far below the tolerance in space, however large it is in radians on so small a radius.
/// Valid every way round, the four results agreeing: |a ∪ b| = |a| + |b| − |a ∩ b| and so on.
#[test]
fn cylinders_micrometres_across_crossing_aslant_combine() {
    // springrep Frame::new(o, z, x) with y = z × x.
    let cyl = |o: [f64; 3], z: [f64; 3], x: [f64; 3], r: f64, h: f64| {
        let f = Frame::new(o, x, cross(z, x), z).unwrap();
        extrude(&f, &Profile::circle([0.0, 0.0], r).unwrap(), h).unwrap()
    };
    let a = cyl([0.0, 0.0, -0.015], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], 0.007173807691792167, 0.03);
    let b = cyl(
        [-0.002532118538179013, 0.0072643840011615635, 0.008785851568869818],
        [0.2734748768191249, -0.9104945125904419, -0.3101793582614065],
        [0.9618791461242749, 0.25886555049919935, 0.08818806617667722],
        0.0053024717958101335,
        0.03,
    );
    let results = [a.union(&b), a.intersection(&b), a.difference(&b), b.difference(&a)];
    let mut vols = Vec::new();
    for r in results {
        let r = r.unwrap();
        assert!(r.check().is_empty(), "{:?}", r.check());
        vols.push(r.volume());
    }
    // Exact to the tolerance in space, so to it times the area in volume.
    let (va, vb) = (a.volume(), b.volume());
    let bound = 1e-7 * (a.area() + b.area());
    assert!((vols[0] - (va + vb - vols[1])).abs() <= bound);
    assert!((vols[2] - (va - vols[1])).abs() <= bound);
    assert!((vols[3] - (vb - vols[1])).abs() <= bound);
}

/// A round thread, a 0.37 mm circle 8.01 mm out, screwed half a turn and a little, cut by a rod on
/// its axis whose side bites through the thread's section: the intersection is the same solid
/// whichever operand comes first, and valid.
#[test]
fn a_thread_and_a_rod_biting_into_it_intersect_either_way_round() {
    // springrep: helix of the circle drawn in the meridian frame (x out, y along z) about z through
    // the origin, right-handed, rising 0.25338… per radian, through 3.19165… rad: yack's screw in
    // the world frame.
    let profile = Profile::circle([8.013653159337178, 0.5917483297087343], 0.37382307626072764).unwrap();
    let thread = screw(&Frame::WORLD, &profile, 0.25338470539470814, rad(3.1916513479467015)).unwrap();
    let rod = rod([0.0, 0.0, -1.0], [0.0, 0.0, 1.0], 7.932446399148578, 3.9922122959395665);
    let ab = thread.intersection(&rod).unwrap();
    let ba = rod.intersection(&thread).unwrap();
    for x in [&ab, &ba] {
        assert!(x.check().is_empty(), "{:?}", x.check());
    }
    // Exact to the tolerance in space, so to it times the area in volume.
    let (va, vb) = (ab.volume(), ba.volume());
    assert!((va - vb).abs() <= 1e-7 * ab.area(), "{va} vs {vb}");
}
