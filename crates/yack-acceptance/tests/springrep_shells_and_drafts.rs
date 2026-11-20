// yack: shells and drafts on flat, cylindrical and spherical walls, against closed forms.
// Adapted from springrep@a03e8a1 tests/shells_and_drafts.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: springrep API replaced by the yack public API (PLAN §4.3); faces named by springrep's
// feature names are picked with Solid::face_at at a point of each, and the four vertical edges with
// Solid::edge_at at their midpoints; edit::shell(.., Wall::Inward) is Solid::shell (positive
// thickness hollows inward, the protocol's convention) and edit::draft on the xy frame is
// Solid::draft with origin 0 and pull +z; the trailing unify is dropped (yack results are already
// merged); springrep circles take a diameter, yack's a radius; springrep's per-face names
// ("block.end", "hollow.inner") are read as yack tags (Solid::named on the op's result, then
// Solid::faces_tagged), the face within a feature told apart by where it lies; outward shells are
// an API gap.

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use yack::prelude::*;
use yack::FaceRef;

/// The 20 × 20 × 10 block on the xy plane, centred on the origin, its faces tagged "block".
fn cube() -> Solid {
    block(&Frame::at([-10.0, -10.0, 0.0]), [20.0, 20.0, 10.0]).unwrap().named("block")
}

/// How many faces tagged `tag` pass `keep`.
fn tagged(b: &Solid, tag: &str, keep: impl Fn(&FaceRef) -> bool) -> usize {
    b.faces_tagged(tag).into_iter().filter(|&f| keep(&b.face(f).unwrap())).count()
}

/// A cylinder R`r` × `h` standing on the xy plane.
fn can(r: f64, h: f64) -> Solid {
    cylinder(&Frame::WORLD, r, h).unwrap()
}

/// `solid` with `faces` tilted by `angle` about where they meet the xy plane.
fn drafted(solid: &Solid, faces: &[FaceId], angle: f64) -> Solid {
    solid.draft(faces, [0.0, 0.0, 0.0], [0.0, 0.0, 1.0], rad(angle)).unwrap()
}

fn faces_of(b: &Solid, keep: impl Fn(SurfaceKind) -> bool) -> usize {
    b.faces_where(|f| keep(f.kind())).len()
}

#[test]
fn a_shell_open_at_the_top_leaves_a_cup() {
    // A 20 × 20 × 10 mm block less a 16 × 16 × 8 cavity open at the top: 4000 − 2048.
    let block = cube();
    let top = block.face_at([0.0, 0.0, 10.0]).unwrap();
    let b = block.shell(&[top], 2.0).unwrap().named("hollow");
    assert!((b.volume() - 1952.0).abs() < 1e-9);
    // "hollow.inner": the cavity's five faces; "block.end": the rim, the block's top where it was.
    assert_eq!(b.faces_tagged("hollow").len(), 5);
    assert_eq!(tagged(&b, "block", |f| (f.centroid().z - 10.0).abs() < 1e-9), 1);
}

// API GAP (test a_shell_grown_outward_keeps_the_inside omitted): Solid::shell hollows inward only
// (the protocol's shell); a wall grown outward round the solid, open at a face, has no §4.3 form.

#[test]
fn a_draft_tapers_the_walls_from_the_neutral_plane() {
    // Two opposite walls of the 20 × 20 × 10 mm block tilted so they come 2 mm in at the top: a
    // trapezoid 20 at the foot and 16 at the top, 10 high, 20 deep.
    let angle = (2.0f64 / 10.0).atan();
    let block = cube();
    let walls = [block.face_at([-10.0, 0.0, 5.0]).unwrap(), block.face_at([10.0, 0.0, 5.0]).unwrap()];
    let b = drafted(&block, &walls, angle);
    let want = 20.0 * 10.0 * 36.0 / 2.0;
    assert!((b.volume() - want).abs() < 1e-9 * want);
    // "block.u_min": the block's face toward −x, now tilted.
    assert_eq!(tagged(&b, "block", |f| f.normal().is_some_and(|n| n.x < -0.9)), 1);
}

#[test]
fn a_cylinder_shelled_open_at_the_top_is_a_cup_with_a_round_wall() {
    // R10 × 20 hollowed 2 mm, the top open: the cavity is the cylinder R8 from z = 2 to 20, its
    // wall the cylinder's offset 2 mm in, exactly.
    let c = can(10.0, 20.0);
    let b = c.shell(&[c.face_at([0.0, 0.0, 20.0]).unwrap()], 2.0).unwrap();
    assert!(b.check().is_empty());
    let want = PI * (100.0 * 20.0 - 64.0 * 18.0);
    assert!((b.volume() - want).abs() < 1e-9 * want, "{}", b.volume());
    let walls: Vec<f64> = b
        .faces()
        .into_iter()
        .filter_map(|f| match b.face(f).unwrap().kind() {
            SurfaceKind::Cylinder { radius, .. } => Some(radius),
            _ => None,
        })
        .collect();
    assert_eq!(walls.len(), 2);
    assert!(walls.iter().any(|r| (r - 8.0).abs() < 1e-12), "{walls:?}");
}

// API GAP (test a_cylinder_shelled_outward_grows_its_wall_round_it omitted): Solid::shell hollows
// inward only; an outward wall open at a face has no §4.3 form.

#[test]
fn a_dome_shelled_open_at_its_base_is_a_spherical_cap() {
    // An R10 hemisphere standing on its flat, hollowed 1 mm with the flat open: the shell between
    // the spheres R10 and R9, 2π(10³ − 9³)/3.
    let (foot, pole) = ([10.0, 0.0], [0.0, 10.0]);
    let quarter = Loop::new(vec![
        Segment::Arc { center: [0.0, 0.0].into(), radius: 10.0, start: rad(0.0), sweep: rad(FRAC_PI_2) },
        Segment::Line { a: pole.into(), b: [0.0, 0.0].into() },
        Segment::Line { a: [0.0, 0.0].into(), b: foot.into() },
    ])
    .unwrap();
    // springrep turns the quarter, drawn on the frame (z = −y, x = x, so its y is world z), about
    // world z; yack's revolve draws in the frame's xz plane and turns about its z.
    let dome = revolve(&Frame::WORLD, &Profile::new(quarter, vec![]).unwrap(), rad(TAU)).unwrap();
    let base = dome.face_at([3.0, 2.0, 0.0]).unwrap();
    let b = dome.shell(&[base], 1.0).unwrap();
    assert!(b.check().is_empty());
    let want = 2.0 * PI * (1000.0 - 729.0) / 3.0;
    assert!((b.volume() - want).abs() < 1e-9 * want, "{}", b.volume());
}

#[test]
fn a_rounded_block_shelled_keeps_its_rounds_as_offset_cylinders() {
    // The 20 × 20 × 10 block with its four vertical edges rounded R4, hollowed 1 mm with the top
    // open. Each section is the rounded square, and the cavity's the square 2 smaller with rounds
    // R3: (20² − (4 − π)·4²)·10 − (18² − (4 − π)·3²)·9.
    let block = cube();
    // The edges between u_min/v_min, v_min/u_max, u_max/v_max and v_max/u_min.
    let corners = [[-10.0, -10.0, 5.0], [10.0, -10.0, 5.0], [10.0, 10.0, 5.0], [-10.0, 10.0, 5.0]];
    let edges: Vec<EdgeId> = corners.iter().map(|&p| block.edge_at(p).unwrap()).collect();
    let rounded = block.fillet(&edges, 4.0).unwrap();
    let b = rounded.shell(&[rounded.face_at([0.0, 0.0, 10.0]).unwrap()], 1.0).unwrap();
    assert!(b.check().is_empty());
    let section = |w: f64, r: f64| w * w - (4.0 - PI) * r * r;
    let want = section(20.0, 4.0) * 10.0 - section(18.0, 3.0) * 9.0;
    assert!((b.volume() - want).abs() < 1e-9 * want, "{}", b.volume());
}

#[test]
fn a_drafted_cylinder_boss_is_a_frustum() {
    // R10 × 10, its side drafted 5° from the base: a frustum from R10 to R10 − 10·tan 5°.
    let c = can(10.0, 10.0);
    let b = drafted(&c, &[c.face_at([10.0, 0.0, 5.0]).unwrap()], 5f64.to_radians());
    assert!(b.check().is_empty());
    let r = 10.0 - 10.0 * 5f64.to_radians().tan();
    let want = PI * 10.0 / 3.0 * (100.0 + 10.0 * r + r * r);
    assert!((b.volume() - want).abs() < 1e-9 * want, "{}", b.volume());
    assert_eq!(faces_of(&b, |k| matches!(k, SurfaceKind::Cone)), 1);
}

#[test]
fn a_drafted_bore_widens_along_the_pull() {
    // The 20 × 20 × 10 block with a Ø8 through hole, the hole's wall drafted 3° from the base: the
    // hole is a frustum from R4 at z = 0 to R4 + 10·tan 3° at the top.
    let outer = Loop::polygon(&[[-10.0, -10.0], [10.0, -10.0], [10.0, 10.0], [-10.0, 10.0]]).unwrap();
    let bore = Loop::new(vec![Segment::Arc {
        center: [0.0, 0.0].into(),
        radius: 8.0 / 2.0,
        start: rad(0.0),
        sweep: rad(-TAU),
    }])
    .unwrap();
    let drilled = extrude(&Frame::WORLD, &Profile::new(outer, vec![bore]).unwrap(), 10.0).unwrap();
    let wall = drilled.face_at([4.0, 0.0, 5.0]).unwrap();
    let b = drafted(&drilled, &[wall], 3f64.to_radians());
    assert!(b.check().is_empty());
    let r = 4.0 + 10.0 * 3f64.to_radians().tan();
    let want = 4000.0 - PI * 10.0 / 3.0 * (16.0 + 4.0 * r + r * r);
    assert!((b.volume() - want).abs() < 1e-9 * want, "{}", b.volume());
}
