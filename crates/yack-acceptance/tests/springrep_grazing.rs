// yack: solids whose curved sides graze — slivers far thinner than long, sides crossing at shallow angles.
// Adapted from springrep@a03e8a1 tests/grazing.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: springrep API replaced by the yack public API (PLAN §4.3); springrep `Frame::new(o, z, x)` written
// as yack `Frame::new(o, x, y, z)` with x made square to z as springrep does and y = z × x; the ring springrep
// transforms after building is built from frames moved by the same `Rigid`s; STEP fixtures read as text.

use std::f64::consts::PI;

use yack::prelude::*;

/// One tooth of each of two spur gears in mesh, cut down to the 1.2 × 1 × 2 mm about where their
/// flanks touch: spline flanks that cross at a few ten-thousandths of a radian, each ending on
/// the same plane as the other.
fn teeth() -> (Solid, Solid) {
    // API GAP: springrep's import lists the entities it rejected and the test asserts there are
    // none; yack's step::read returns only the solid.
    (
        yack::step::read(include_str!("fixtures/grazing-tooth-a.step")).unwrap(),
        yack::step::read(include_str!("fixtures/grazing-tooth-b.step")).unwrap(),
    )
}

#[test]
fn two_flanks_that_graze_share_the_same_sliver_whichever_is_first() {
    // Where two edges cross at a shallow angle the crossing is found from each solid's edge, a
    // few tenths of a micron apart along the graze: it is one vertex, and every intersection
    // curve of the faces round it ends there.
    let (a, b) = teeth();
    let (one, other) = (a.intersection(&b).unwrap(), b.intersection(&a).unwrap());
    assert!(one.check().is_empty() && other.check().is_empty());
    let (v, w) = (one.volume(), other.volume());
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
    let shaft = cylinder(&Frame::at([0.0, 0.0, -5.0]), 2.0, 20.0).unwrap();
    for (d, tilt) in [(4e-4, 0.0), (1e-5, 0.0), (4e-4, 1.6e-5)] {
        // springrep turns the ring (outer less bore) by `tilt` about x through (0, 0, 4), then moves
        // it d along y; here outer and bore are built in frames so moved, then differenced.
        let rot = Rigid::rotation([0.0, 0.0, 4.0], [1.0, 0.0, 0.0], rad(tilt)).unwrap();
        let shift = Rigid::translation([0.0, d, 0.0]);
        let place = |f: Frame| f.moved(&rot).moved(&shift);
        let outer = cylinder(&place(Frame::WORLD), 2.9, 8.0).unwrap();
        let bore = cylinder(&place(Frame::at([0.0, 0.0, -1.0])), 2.0, 10.0).unwrap();
        let placed = outer.difference(&bore).unwrap();
        let shared = shaft.intersection(&placed).unwrap_or_else(|e| panic!("{d} {tilt}: {e}"));
        assert!(shared.check().is_empty(), "{d} {tilt}");
        // 2·r·d·8, less a d³ term far under the tolerance; a tilt this small changes it by
        // under a part in a million.
        let want = 2.0 * 2.0 * d * 8.0;
        let got = shared.volume();
        assert!((got - want).abs() < 1e-5 * want, "{d} {tilt}: {got} against {want}");
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
        [r * a.cos(), r * a.sin(), 0.0]
    };
    let s = 0.5f64.sqrt();
    // out = (1, 1, 0) / √2; springrep frames (o, Z, ±out) and (o, Z, −X), y = Z × x.
    let disc = cylinder(&Frame::new([0.0; 3], [s, s, 0.0], [-s, s, 0.0], [0.0, 0.0, 1.0]).unwrap(), 42.5, 16.0).unwrap();
    let n = along(43.0 + 3.16e-9);
    let notch = cylinder(
        &Frame::new([n[0], n[1], n[2] - 1.0], [-s, -s, 0.0], [s, -s, 0.0], [0.0, 0.0, 1.0]).unwrap(),
        2.0,
        18.0,
    )
    .unwrap();
    let notched = disc.difference(&notch).unwrap();
    let rib = cylinder(&Frame::new(along(42.95), [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]).unwrap(), 1.95, 20.0)
        .unwrap();
    let shared = notched.intersection(&rib).unwrap();
    assert!(shared.check().is_empty());
    assert!(shared.volume().abs() < 1e-12, "{}", shared.volume());
}

/// springrep `Frame::new(o, Y, Z)`: x = Z, y = Y × Z = X.
fn y_up_z_across(o: [f64; 3]) -> Frame {
    Frame::new(o, [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]).unwrap()
}

/// springrep `Frame::new(o, Y, X)`: x = X, y = Y × X = −Z.
fn y_up_x_across(o: [f64; 3]) -> Frame {
    Frame::new(o, [1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]).unwrap()
}

#[test]
fn a_press_fit_pin_a_hair_off_its_bores_axis_shares_the_ring_of_its_interference() {
    // A plate 4.7 thick bored r 3.95, and a pin r 4 pressed into it with its end flush with the
    // plate's face, chamfered 0.5 at 45 degrees, its axis 3e-7 off the bore's: more than the
    // tolerance, so not on one axis, and far less than anything a design means. The chamfer
    // crosses the bore in a curve that is a circle to the tolerance all round.
    let plate = block(&y_up_z_across([-10.0, 0.0, -10.0]), [20.0, 20.0, 4.7]).unwrap();
    let bore = cylinder(&y_up_x_across([0.0, -1.0, 0.0]), 3.95, 10.0).unwrap();
    let holed = plate.difference(&bore).unwrap();
    let o = 3e-7;
    let chamfer = cone(&y_up_x_across([o, 0.0, 0.0]), 3.5, 4.0, 0.5).unwrap();
    let shank = cylinder(&y_up_x_across([o, 0.5, 0.0]), 4.0, 20.0).unwrap();
    let pin = chamfer.union(&shank).unwrap();
    let shared = holed.intersection(&pin).unwrap();
    assert!(shared.check().is_empty());
    // The ring between r 3.95 and 4 over the 4.2 of shank in the plate, and over the chamfer the
    // ring between r 3.95 and its radius from y = 0.45 to 0.5: π·(4² − 3.95²)·4.2 plus the cone's
    // π·∫ (r(y)² − 3.95²) dy with r = 3.5 + y, from 0.45 to 0.5: π·0.05²·0.05/3... worked as
    // π·(0.05³/3 + 3.95·0.05²) = π·0.009917.
    let want = PI * (16.0 - 15.6025) * 4.2 + PI * (0.05f64.powi(3) / 3.0 + 3.95 * 0.0025);
    let got = shared.volume();
    assert!((got - want).abs() < 1e-6 * want, "{got} against {want}");
}

/// The plate 8 thick bored r 2.9 with a chamfered mouth, of the two insert tests.
fn holed_plate() -> Solid {
    let plate = block(&y_up_z_across([-10.0, 0.0, -10.0]), [20.0, 20.0, 8.0]).unwrap();
    let bore = cylinder(&y_up_x_across([0.0, -1.0, 0.0]), 2.9, 10.0).unwrap();
    let mouth = cone(&y_up_x_across([0.0, -0.5, 0.0]), 3.9, 2.9, 1.0).unwrap();
    plate.difference(&bore).unwrap().difference(&mouth).unwrap()
}

#[test]
fn an_insert_turned_a_hair_off_its_holes_axis_shares_the_crescent_between_them() {
    // A plate 8 thick bored r 2.9 with a chamfered mouth, and an insert r 2.9 in the bore, its end
    // at the chamfer's foot, its axis 5.2e-6 off the bore's across the seams and turned 2e-10 off
    // it, as a rounded pose leaves it. Parallel, the cylinders cross in two lines along both seams;
    // turned that hair, in curves running along each other the whole length.
    let holed = holed_plate();
    let d = 5.2e-6;
    for lift in [0.0, -4.2e-6, 0.3] {
        // springrep Frame::new(o, (2e-10, 1, 0), −X): x = −X made square to z = (−1, 2e-10, 0),
        // y = z × x = Z.
        let frame = Frame::new([0.0, 0.5 + lift, d], [-1.0, 2e-10, 0.0], [0.0, 0.0, 1.0], [2e-10, 1.0, 0.0]).unwrap();
        let insert = cylinder(&frame, 2.9, 6.0).unwrap();
        let shared = holed.intersection(&insert).unwrap_or_else(|e| panic!("{lift}: {e}"));
        assert!(shared.check().is_empty(), "{lift}");
        // The crescent 2·r·d across, over the 6 of insert in the bore.
        let want = 2.0 * 2.9 * d * 6.0;
        let got = shared.volume();
        assert!((got - want).abs() < 1e-3 * want, "{lift}: {got} against {want}");
    }
}

#[test]
fn an_insert_a_hair_off_its_holes_axis_with_its_end_a_hair_into_the_chamfer_is_valid() {
    // The plate of the test above, and the insert 5.2e-6 off the bore's axis at 5° to x, its end
    // 4.2e-6 out of the bore into the chamfer, cut flat at z = 2. The insert's rim crosses the
    // chamfer at about 1e-6 rad, placed only to a tenth of a millimetre, and the cut's corner on
    // the rim lies 0.14 mm from it. On one axis, the insert only touches the bore.
    let holed = holed_plate();
    let below = block(&Frame::at([-10.0, -1.0, -10.0]), [20.0, 12.0, 12.0]).unwrap();
    let d = 5.2e-6;
    let (s, c) = 5f64.to_radians().sin_cos();
    // springrep Frame::new(o, Y, −X): x = −X, y = Y × −X = Z.
    let frame = Frame::new([d * c, 0.5 - 4.2e-6, d * s], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]).unwrap();
    let insert = cylinder(&frame, 2.9, 6.0).unwrap();
    let insert = insert.intersection(&below).unwrap();
    let shared = holed.intersection(&insert).unwrap();
    assert!(shared.check().is_empty());
    // At most the crescent 2·r·d across over the 6 of insert.
    let got = shared.volume();
    assert!(got.abs() <= 2.0 * 2.9 * d * 6.0, "{got}");
}

#[test]
fn a_plane_a_hair_off_parallel_to_a_holes_axis_cuts_it_in_two_lines() {
    // A plate 10 × 10 × 4 with a hole r 1.6 through it, its axis turned 5.7e-10 rad off z as a
    // rounded pose leaves it, cut at y = 5.5, through the hole 0.5 off its axis. The plane meets
    // the hole in an ellipse 2.8e9 mm long whose centre lies 1e9 mm away; across the plate it is
    // two lines to far under the tolerance.
    let plate = block(&Frame::WORLD, [10.0, 10.0, 4.0]).unwrap();
    let tilt = 5.7e-10;
    // springrep Frame::new(o, (0, tilt, 1), X): x = X, y = z × x = (0, 1, −tilt).
    let hole = cylinder(&Frame::new([5.0, 5.0, -1.0], [1.0, 0.0, 0.0], [0.0, 1.0, -tilt], [0.0, tilt, 1.0]).unwrap(), 1.6, 6.0)
        .unwrap();
    let holed = plate.difference(&hole).unwrap();
    let below = block(&Frame::at([-1.0, -1.0, -1.0]), [12.0, 6.5, 6.0]).unwrap();
    let cut = holed.intersection(&below).unwrap();
    assert!(cut.check().is_empty());
    // The plate's 10 × 5.5 × 4, less the hole's disc below y = 5.5 over the 4: the whole disc but
    // the segment beyond a chord 0.5 off its centre, r²·acos(d/r) − d·√(r² − d²).
    let (r, d): (f64, f64) = (1.6, 0.5);
    let segment = r * r * (d / r).acos() - d * (r * r - d * d).sqrt();
    let want = 220.0 - 4.0 * (PI * r * r - segment);
    let got = cut.volume();
    assert!((got - want).abs() < 1e-9 * want, "{got} against {want}");
}
