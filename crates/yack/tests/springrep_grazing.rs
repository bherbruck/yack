// yack: Solids whose curved sides graze: they share a sliver far thinner than it is long, their sides
// Adapted from springrep@a03e8a1 tests/grazing.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: crate paths springrep:: -> yack_springrep::; this header added. Moved to crates/yack/tests at W11 (springrep deleted): paths to the crates the items live in.
//! Solids whose curved sides graze: they share a sliver far thinner than it is long, their sides
//! cross at a shallow angle, and where the sliver ends each solid's edges cross the other's.
// W11: springrep's own kernel tests, moved here when springrep went (ruling (b)). They build with the
// kernel crates directly and compute their expected values, so the facade's maths ban and the
// workspace lints' test-only exemptions (which reach only #[test] fns) do not fit them.
// TODO(api-owner, hardening, D26): rewrite these tests to the lints, test by test, and drop this allow.
#![allow(clippy::disallowed_methods, clippy::disallowed_types, clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::indexing_slicing)]

use std::f64::consts::PI;

use yack_boolean::difference;
use yack_boolean::intersection;
use yack_boolean::union;
use yack_check as check;
use yack_math::kernel::Frame;
use yack_math::kernel::Point3;
use yack_math::kernel::Transform;
use yack_math::kernel::Vec3;
use yack_measure as measure;
use yack_build::primitive;
use yack_step as step;
use yack_topo::Brep;

/// One tooth of each of two spur gears in mesh, cut down to the 1.2 × 1 × 2 mm about where their
/// flanks touch: spline flanks that cross at a few ten-thousandths of a radian, each ending on
/// the same plane as the other.
fn teeth() -> (Brep, Brep) {
    let read = |bytes: &[u8], name: &str| {
        let import = step::read(bytes, name).unwrap();
        assert!(import.rejected.is_empty(), "{:?}", import.rejected);
        import.brep
    };
    (
        read(include_bytes!("fixtures/grazing-tooth-a.step"), "a"),
        read(include_bytes!("fixtures/grazing-tooth-b.step"), "b"),
    )
}

#[test]
fn two_flanks_that_graze_share_the_same_sliver_whichever_is_first() {
    // Where two edges cross at a shallow angle the crossing is found from each solid's edge, a
    // few tenths of a micron apart along the graze: it is one vertex, and every intersection
    // curve of the faces round it ends there.
    let (a, b) = teeth();
    let (one, other) = (intersection(&a, &b).unwrap(), intersection(&b, &a).unwrap());
    assert!(check::check(&one).is_empty() && check::check(&other).is_empty());
    let (v, w) = (measure::volume(&one), measure::volume(&other));
    // A lens about 2.4e-6 mm³: 0.4 mm³ and 0.7 mm³ of tooth sharing a few millionths.
    assert!((2.0e-6..3.0e-6).contains(&v), "{v}");
    assert!((v - w).abs() < 1e-3 * v, "{v} against {w}");
}

#[test]
fn a_shaft_a_hair_off_its_bores_axis_shares_the_crescent_between_them() {
    // A shaft r 2 and a ring whose bore is r 2, 8 long, the bore's axis d off the shaft's: the
    // shaft stands out of the bore on one side by up to d, a crescent of area 2·r·d to first
    // order. The bore's seam lies along the side the shaft stands out least, d²/(2r) from the
    // shaft's surface, inside the tolerance once d is under half a micron: it bounds no material,
    // and the shaft is cut only where the two surfaces cross.
    let shaft = primitive::cylinder(
        &Frame::new(Vec3::new(0.0, 0.0, -5.0), Vec3::Z, Vec3::X).unwrap(),
        2.0,
        20.0,
        "shaft",
    )
    .unwrap();
    let outer = primitive::cylinder(&Frame::WORLD, 2.9, 8.0, "outer").unwrap();
    let bore = primitive::cylinder(
        &Frame::new(Vec3::new(0.0, 0.0, -1.0), Vec3::Z, Vec3::X).unwrap(),
        2.0,
        10.0,
        "bore",
    )
    .unwrap();
    let ring = difference(&outer, &bore).unwrap();
    for (d, tilt) in [(4e-4, 0.0), (1e-5, 0.0), (4e-4, 1.6e-5)] {
        let mut placed = ring.clone();
        placed.transform(&Transform::rotation(
            Point3::new(0.0, 0.0, 4.0),
            Vec3::X,
            tilt,
        ));
        placed.transform(&Transform::translation(Vec3::new(0.0, d, 0.0)));
        let shared = intersection(&shaft, &placed).unwrap_or_else(|e| panic!("{d} {tilt}: {e}"));
        assert!(check::check(&shared).is_empty(), "{d} {tilt}");
        // 2·r·d·8, less a d³ term far under the tolerance; a tilt this small changes it by
        // under a part in a million.
        let want = 2.0 * 2.0 * d * 8.0;
        let got = measure::volume(&shared);
        assert!(
            (got - want).abs() < 1e-5 * want,
            "{d} {tilt}: {got} against {want}"
        );
    }
}

#[test]
fn a_rib_a_hair_into_the_notch_it_stands_in_only_touches_it() {
    // A disc r 42.5, 16 thick, with a notch r 2 in its rim, its axis at r 43 round 225 degrees;
    // a rib r 1.95 stands in the notch, its axis at r 42.95 on the same line, so the two touch
    // along one line at r 41. The notch is set 3.16e-9 further out, as a rounded pose leaves it:
    // the rib reaches that far into the disc. Curvatures this close cross 1.4 um either side of
    // where they touch for so small an overlap, and the sliver between has no side; within the
    // tolerance the rib touches the notch, and they share nothing.
    let along = |r: f64| {
        let a = 1.25 * PI;
        Vec3::new(r * a.cos(), r * a.sin(), 0.0)
    };
    let out = Vec3::new(1.0, 1.0, 0.0).unit().unwrap();
    let disc = primitive::cylinder(
        &Frame::new(Vec3::ZERO, Vec3::Z, out).unwrap(),
        42.5,
        16.0,
        "disc",
    )
    .unwrap();
    let notch = primitive::cylinder(
        &Frame::new(along(43.0 + 3.16e-9) - Vec3::Z, Vec3::Z, -out).unwrap(),
        2.0,
        18.0,
        "notch",
    )
    .unwrap();
    let notched = difference(&disc, &notch).unwrap();
    let rib = primitive::cylinder(
        &Frame::new(along(42.95), Vec3::Z, -Vec3::X).unwrap(),
        1.95,
        20.0,
        "rib",
    )
    .unwrap();
    let shared = intersection(&notched, &rib).unwrap();
    assert!(check::check(&shared).is_empty());
    assert!(
        measure::volume(&shared).abs() < 1e-12,
        "{}",
        measure::volume(&shared)
    );
}

#[test]
fn a_press_fit_pin_a_hair_off_its_bores_axis_shares_the_ring_of_its_interference() {
    // A plate 4.7 thick bored r 3.95, and a pin r 4 pressed into it with its end flush with the
    // plate's face, chamfered 0.5 at 45 degrees, its axis 3e-7 off the bore's: more than the
    // tolerance, so not on one axis, and far less than anything a design means. The chamfer
    // crosses the bore in a curve that is a circle to the tolerance all round.
    let plate = primitive::block(
        &Frame::new(Vec3::new(-10.0, 0.0, -10.0), Vec3::Y, Vec3::Z).unwrap(),
        Vec3::new(20.0, 20.0, 4.7),
        "plate",
    )
    .unwrap();
    let bore = primitive::cylinder(
        &Frame::new(Vec3::new(0.0, -1.0, 0.0), Vec3::Y, Vec3::X).unwrap(),
        3.95,
        10.0,
        "bore",
    )
    .unwrap();
    let holed = difference(&plate, &bore).unwrap();
    let o = Vec3::new(3e-7, 0.0, 0.0);
    let chamfer = primitive::cone(
        &Frame::new(o, Vec3::Y, Vec3::X).unwrap(),
        3.5,
        4.0,
        0.5,
        "pin",
    )
    .unwrap();
    let shank = primitive::cylinder(
        &Frame::new(o + Vec3::new(0.0, 0.5, 0.0), Vec3::Y, Vec3::X).unwrap(),
        4.0,
        20.0,
        "pin",
    )
    .unwrap();
    let pin = union(&chamfer, &shank).unwrap();
    let shared = intersection(&holed, &pin).unwrap();
    assert!(check::check(&shared).is_empty());
    // The ring between r 3.95 and 4 over the 4.2 of shank in the plate, and over the chamfer the
    // ring between r 3.95 and its radius from y = 0.45 to 0.5: π·(4² − 3.95²)·4.2 plus the cone's
    // π·∫ (r(y)² − 3.95²) dy with r = 3.5 + y, from 0.45 to 0.5: π·0.05²·0.05/3... worked as
    // π·(0.05³/3 + 3.95·0.05²) = π·0.009917.
    let want = PI * (16.0 - 15.6025) * 4.2 + PI * (0.05f64.powi(3) / 3.0 + 3.95 * 0.0025);
    let got = measure::volume(&shared);
    assert!((got - want).abs() < 1e-6 * want, "{got} against {want}");
}

#[test]
fn an_insert_turned_a_hair_off_its_holes_axis_shares_the_crescent_between_them() {
    // A plate 8 thick bored r 2.9 with a chamfered mouth, and an insert r 2.9 in the bore, its end
    // at the chamfer's foot, its axis 5.2e-6 off the bore's across the seams and turned 2e-10 off
    // it, as a rounded pose leaves it. Parallel, the cylinders cross in two lines along both seams;
    // turned that hair, in curves running along each other the whole length.
    let plate = primitive::block(
        &Frame::new(Vec3::new(-10.0, 0.0, -10.0), Vec3::Y, Vec3::Z).unwrap(),
        Vec3::new(20.0, 20.0, 8.0),
        "plate",
    )
    .unwrap();
    let bore = primitive::cylinder(
        &Frame::new(Vec3::new(0.0, -1.0, 0.0), Vec3::Y, Vec3::X).unwrap(),
        2.9,
        10.0,
        "bore",
    )
    .unwrap();
    let mouth = primitive::cone(
        &Frame::new(Vec3::new(0.0, -0.5, 0.0), Vec3::Y, Vec3::X).unwrap(),
        3.9,
        2.9,
        1.0,
        "mouth",
    )
    .unwrap();
    let holed = difference(&difference(&plate, &bore).unwrap(), &mouth).unwrap();
    let d = 5.2e-6;
    for lift in [0.0, -4.2e-6, 0.3] {
        let insert = primitive::cylinder(
            &Frame::new(
                Vec3::new(0.0, 0.5 + lift, d),
                Vec3::new(2e-10, 1.0, 0.0),
                -Vec3::X,
            )
            .unwrap(),
            2.9,
            6.0,
            "insert",
        )
        .unwrap();
        let shared = intersection(&holed, &insert).unwrap_or_else(|e| panic!("{lift}: {e}"));
        assert!(check::check(&shared).is_empty(), "{lift}");
        // The crescent 2·r·d across, over the 6 of insert in the bore.
        let want = 2.0 * 2.9 * d * 6.0;
        let got = measure::volume(&shared);
        assert!(
            (got - want).abs() < 1e-3 * want,
            "{lift}: {got} against {want}"
        );
    }
}

#[test]
fn an_insert_a_hair_off_its_holes_axis_with_its_end_a_hair_into_the_chamfer_is_valid() {
    // The plate of the test above, and the insert 5.2e-6 off the bore's axis at 5° to x, its end
    // 4.2e-6 out of the bore into the chamfer, cut flat at z = 2. The insert's rim crosses the
    // chamfer at about 1e-6 rad, placed only to a tenth of a millimetre, and the cut's corner on
    // the rim lies 0.14 mm from it. On one axis, the insert only touches the bore.
    let plate = primitive::block(
        &Frame::new(Vec3::new(-10.0, 0.0, -10.0), Vec3::Y, Vec3::Z).unwrap(),
        Vec3::new(20.0, 20.0, 8.0),
        "plate",
    )
    .unwrap();
    let bore = primitive::cylinder(
        &Frame::new(Vec3::new(0.0, -1.0, 0.0), Vec3::Y, Vec3::X).unwrap(),
        2.9,
        10.0,
        "bore",
    )
    .unwrap();
    let mouth = primitive::cone(
        &Frame::new(Vec3::new(0.0, -0.5, 0.0), Vec3::Y, Vec3::X).unwrap(),
        3.9,
        2.9,
        1.0,
        "mouth",
    )
    .unwrap();
    let holed = difference(&difference(&plate, &bore).unwrap(), &mouth).unwrap();
    let below = primitive::block(
        &Frame::new(Vec3::new(-10.0, -1.0, -10.0), Vec3::Z, Vec3::X).unwrap(),
        Vec3::new(20.0, 12.0, 12.0),
        "below",
    )
    .unwrap();
    let d = 5.2e-6;
    let (s, c) = 5f64.to_radians().sin_cos();
    let insert = primitive::cylinder(
        &Frame::new(Vec3::new(d * c, 0.5 - 4.2e-6, d * s), Vec3::Y, -Vec3::X).unwrap(),
        2.9,
        6.0,
        "insert",
    )
    .unwrap();
    let insert = intersection(&insert, &below).unwrap();
    let shared = intersection(&holed, &insert).unwrap();
    assert!(check::check(&shared).is_empty());
    // At most the crescent 2·r·d across over the 6 of insert.
    let got = measure::volume(&shared);
    assert!(got.abs() <= 2.0 * 2.9 * d * 6.0, "{got}");
}

#[test]
fn a_plane_a_hair_off_parallel_to_a_holes_axis_cuts_it_in_two_lines() {
    // A plate 10 × 10 × 4 with a hole r 1.6 through it, its axis turned 5.7e-10 rad off z as a
    // rounded pose leaves it, cut at y = 5.5, through the hole 0.5 off its axis. The plane meets
    // the hole in an ellipse 2.8e9 mm long whose centre lies 1e9 mm away; across the plate it is
    // two lines to far under the tolerance.
    let plate = primitive::block(&Frame::WORLD, Vec3::new(10.0, 10.0, 4.0), "plate").unwrap();
    let tilt = 5.7e-10;
    let hole = primitive::cylinder(
        &Frame::new(
            Vec3::new(5.0, 5.0, -1.0),
            Vec3::new(0.0, tilt, 1.0),
            Vec3::X,
        )
        .unwrap(),
        1.6,
        6.0,
        "hole",
    )
    .unwrap();
    let holed = difference(&plate, &hole).unwrap();
    let below = primitive::block(
        &Frame::new(Vec3::new(-1.0, -1.0, -1.0), Vec3::Z, Vec3::X).unwrap(),
        Vec3::new(12.0, 6.5, 6.0),
        "below",
    )
    .unwrap();
    let cut = intersection(&holed, &below).unwrap();
    assert!(check::check(&cut).is_empty());
    // The plate's 10 × 5.5 × 4, less the hole's disc below y = 5.5 over the 4: the whole disc but
    // the segment beyond a chord 0.5 off its centre, r²·acos(d/r) − d·√(r² − d²).
    let (r, d): (f64, f64) = (1.6, 0.5);
    let segment = r * r * (d / r).acos() - d * (r * r - d * d).sqrt();
    let want = 220.0 - 4.0 * (PI * r * r - segment);
    let got = measure::volume(&cut);
    assert!((got - want).abs() < 1e-9 * want, "{got} against {want}");
}
