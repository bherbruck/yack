// yack: lofts between square sections (Cavalieri volume) and faces pushed along their normals.
// Adapted from springrep@a03e8a1 tests/lofts_and_thickens.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: springrep API replaced by the yack public API (PLAN §4.3); springrep's "thicken" helper
// (edit::offset_faces then unify) is Solid::offset_faces (protocol `offset_faces`, which merges
// faces itself; protocol `thicken` is a different operation); faces named "block.u_max" etc. are
// picked with face_at, and "hole.wall" by its cylinder kind; one test omitted (API GAP below).

//! Ruled lofts through square sections against the frustum and Cavalieri volumes, and faces
//! thickened by moving them along their outward normals, flat walls and a bore's cylinder.

use std::f64::consts::{PI, TAU};

use yack::prelude::*;
use yack::{FaceId, SurfaceKind};

/// A `w` × `w` square centred on (`cx`, 0).
fn square(cx: f64, w: f64) -> Profile {
    Profile::rectangle([cx - w / 2.0, -w / 2.0], [cx + w / 2.0, w / 2.0]).unwrap()
}

/// The 20 × 20 × 10 block on the xy plane, centred on the origin.
fn block() -> Solid {
    extrude(&Frame::WORLD, &square(0.0, 20.0), 10.0).unwrap()
}

/// `solid` with each face in `faces` moved `d` along its outward normal.
fn thickened(solid: &Solid, faces: &[FaceId], d: f64) -> Solid {
    assert!(!faces.is_empty(), "no face picked");
    solid.offset_faces(faces, d).unwrap()
}

/// The area of the one face through `p`.
fn area_at(b: &Solid, p: [f64; 3]) -> f64 {
    b.face(b.face_at(p).unwrap()).unwrap().area()
}

fn bore_faces(b: &Solid) -> Vec<FaceId> {
    b.faces_where(|f| matches!(f.kind(), SurfaceKind::Cylinder { .. }))
}

// API GAP (test a_ruled_loft_through_three_sections_is_two_frustums omitted): the closed form
// (two square frustums) needs a ruled loft through three sections; §4.3 `loft_through` is the
// protocol's smooth loft ("how the side runs between sections is open") and `loft` takes only two
// sections, a scaled copy.

#[test]
fn a_ruled_loft_between_shifted_equal_sections_holds_their_area_all_the_way() {
    // Two 10 mm squares 10 apart, the top one 5 along x: a leaning prism, every level the same
    // square, so by Cavalieri 100 · 10.
    // yack: two sections through `loft_through`; with two sections the side runs straight between
    // them. API GAP: §4.3 has no ruled flag, so this relies on a two-section loft being ruled.
    let b = loft_through(&Frame::WORLD, &[(0.0, square(0.0, 10.0)), (10.0, square(5.0, 10.0))]).unwrap();
    assert!(b.check().is_empty());
    assert!((b.volume() - 1000.0).abs() < 1e-9 * 1000.0, "{}", b.volume());
}

#[test]
fn thickening_a_wall_moves_it_out_and_stretches_the_faces_beside_it() {
    // The 20 × 20 × 10 block's +x wall moved out 2: a 22 × 20 × 10 block whose top is 22 × 20.
    let blk = block();
    let b = thickened(&blk, &[blk.face_at([10.0, 0.0, 5.0]).unwrap()], 2.0);
    assert!((b.volume() - 4400.0).abs() < 1e-9 * 4400.0);
    assert_eq!(b.faces().len(), 6);
    assert!((area_at(&b, [0.0, 0.0, 10.0]) - 440.0).abs() < 1e-9);
    assert!((area_at(&b, [12.0, 0.0, 5.0]) - 200.0).abs() < 1e-9);
    // Negative takes material away: both side walls in 1 leaves 18 × 20 × 10.
    let walls = [blk.face_at([-10.0, 0.0, 5.0]).unwrap(), blk.face_at([10.0, 0.0, 5.0]).unwrap()];
    let b = thickened(&blk, &walls, -1.0);
    assert!((b.volume() - 3600.0).abs() < 1e-9 * 3600.0);
    assert!((area_at(&b, [0.0, 0.0, 0.0]) - 360.0).abs() < 1e-9);
}

#[test]
fn thickening_a_bore_closes_it_and_thinning_it_opens_it() {
    // A Ø6 hole through the 10 mm block: its wall's outward normal points into the hole, so +1
    // leaves a Ø4 bore and −1 a Ø8 one.
    let hole = extrude(&Frame::WORLD, &Profile::circle([0.0, 0.0], 3.0).unwrap(), 10.0).unwrap();
    let drilled = block().difference(&hole).unwrap();
    for (d, r) in [(1.0, 2.0), (-1.0, 4.0)] {
        let b = thickened(&drilled, &bore_faces(&drilled), d);
        let want = 4000.0 - PI * r * r * 10.0;
        assert!((b.volume() - want).abs() < 1e-9 * want, "{d}");
        let wall = bore_faces(&b);
        assert_eq!(wall.len(), 1, "hole.wall");
        assert!((b.face(wall[0]).unwrap().area() - TAU * r * 10.0).abs() < 1e-8, "{d}");
    }
}
