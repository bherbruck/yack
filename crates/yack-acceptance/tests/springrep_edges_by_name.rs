// yack: chamfers and fillets of edges where two picked faces meet: swept, revolved, keyed and overhanging parts.
// Adapted from springrep@a03e8a1 tests/edges_by_name.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: springrep API replaced by the yack public API (PLAN §4.3); edges named by face-name pairs are
// selected with `face_at` + `edges_between` (exact pairs, as springrep's `EdgeName { exact: true }`);
// springrep's centred `Profile::rect` is written as `Profile::rectangle(min, max)`, `named_circle(c, 8.0)`
// (a diameter) as `Profile::circle(c, 4.0)`; the revolve about the sketch's y axis on a frame whose xy is
// the world xz plane is yack's `revolve` on `Frame::WORLD` (profile (ρ, z) turned about z); `unify` has no
// §4.3 counterpart and is dropped (it only preceded checks and measures); "watertight" is
// `Mesh::is_watertight` at chord 0.01; a feature's face count is `named` + `faces_tagged` (yack tags are
// feature names; the per-side names a profile gives, such as `shaft.face_a`, are still an API gap).
//! Chamfers and fillets whose edges are picked by the names of the two faces that meet in them:
//! names a sweep gave from its profile, names on a revolved solid, an edge a keyway interrupts,
//! and a pair of surfaces that meet in more than one edge.

use std::f64::consts::{PI, TAU};

use yack::SurfaceKind;
use yack::prelude::*;

/// How far a through hole runs, so the cut leaves no skin.
const THROUGH: f64 = 1000.0;

/// A `width` × `width` block 10 tall on the xy plane, centred on the origin.
fn block(width: f64) -> Solid {
    let h = width / 2.0;
    extrude(&Frame::WORLD, &Profile::rectangle([-h, -h], [h, h]).unwrap(), 10.0).unwrap()
}

/// Every edge between the face through `p` and the face through `q`.
fn between(b: &Solid, p: [f64; 3], q: [f64; 3]) -> Vec<EdgeId> {
    b.edges_between(b.face_at(p).unwrap(), b.face_at(q).unwrap()).unwrap()
}

fn watertight(b: &Solid) -> bool {
    // springrep: MeshOptions { linear: 0.01, angular: 0.2 } is chord 0.01.
    b.mesh(0.01).expect("the result meshes").is_watertight()
}

fn cylinders(b: &Solid) -> usize {
    b.faces_where(|f| matches!(f.kind(), SurfaceKind::Cylinder { .. })).len()
}

#[test]
fn a_chamfer_bevels_the_named_edge() {
    let cube = extrude(&Frame::WORLD, &Profile::rectangle([-2.0, -2.0], [2.0, 2.0]).unwrap(), 4.0).unwrap();
    // (block.end, block.u_max)
    let b = cube.chamfer(&between(&cube, [0.0, 0.0, 4.0], [2.0, 0.0, 2.0]), 1.0).unwrap().named("ch");
    let got = b.volume();
    assert!((got - 62.0).abs() < 1e-9 * 62.0, "{got}");
    // "ch.bevel_0" names one face.
    assert_eq!(b.faces_tagged("ch").len(), 1);
}

#[test]
fn a_chamfer_picks_a_revolved_shafts_edge_by_its_profile_names() {
    // A 10 mm radius, 10 mm long shaft turned from a rectangle, its end ring bevelled 1 mm: the
    // bevel takes a 1 × 1 / 2 triangle whose centroid turns at 10 − 1/3 (Pappus).
    // The outline's sides: face_a (z = 0), journal (ρ = 10), face_b (z = 10), axis.
    let outline = Profile::rectangle([0.0, 0.0], [10.0, 10.0]).unwrap();
    let shaft = revolve(&Frame::WORLD, &outline, rad(TAU)).unwrap();
    // (shaft.face_a, shaft.journal)
    let b = shaft.chamfer(&between(&shaft, [0.0, 5.0, 0.0], [0.0, 10.0, 5.0]), 1.0).unwrap().named("break");
    let want = 1000.0 * PI - 2.0 * PI * (10.0 - 1.0 / 3.0) * 0.5;
    assert!((b.volume() - want).abs() < 1e-9 * want);
    // "break.bevel_0" names one face.
    assert_eq!(b.faces_tagged("break").len(), 1);
    // API GAP: "shaft.face_a" and "shaft.journal" each name one face; needs the outline's side names
    // (`Pen::label`) carried onto faces as tags.
}

/// A 20 × 20 × 10 plate with a ⌀8 bore through it and a keyway 2 wide cut 1 past the bore's wall
/// on +x, the bore's edge on the plate's underside bevelled 0.5 when `chamfered`.
fn keyed_bore(chamfered: bool) -> Solid {
    let plate = extrude(&Frame::WORLD, &Profile::rectangle([-10.0, -10.0], [10.0, 10.0]).unwrap(), 10.0).unwrap();
    let bore = extrude(&Frame::WORLD, &Profile::circle([0.0, 0.0], 4.0).unwrap(), THROUGH).unwrap();
    let key = extrude(&Frame::WORLD, &Profile::rectangle([3.5, -1.0], [5.0, 1.0]).unwrap(), 10.0).unwrap();
    let mut b = plate.difference(&bore).unwrap().difference(&key).unwrap();
    if chamfered {
        // (bore.wall, plate.start)
        b = b.chamfer(&between(&b, [-4.0, 0.0, 5.0], [-8.0, 0.0, 0.0]), 0.5).unwrap();
    }
    b
}

#[test]
fn a_chamfer_round_most_of_a_bore_stops_at_the_keyway_walls() {
    // The bore's edge runs from one keyway wall (y = 1) round to the other (y = −1), 331°. The
    // chamfer's triangle, 0.5 mm each way, turns everywhere the keyway leaves material: at radius
    // r it is 4.5 − r tall and misses the 2·asin(1/r) the keyway takes. Each end is trimmed by
    // its own wall, which the far end lies across.
    let removed = keyed_bore(false).volume() - keyed_bore(true).volume();
    let n = 2000;
    let f = |r: f64| (4.5 - r) * r * (TAU - 2.0 * (1.0 / r).asin());
    let h = 0.5 / n as f64;
    let mut want = f(4.0) + f(4.5);
    for i in 1..n {
        want += f(4.0 + h * i as f64) * if i % 2 == 1 { 4.0 } else { 2.0 };
    }
    want *= h / 3.0;
    assert!((removed - want).abs() < 1e-9 * want, "{removed} vs {want}");
}

/// The block with a 10 × 10 × 5 boss on its top from x = 5 to 15, overhanging its +x side: at
/// (10, ±5, 10) the block's top, the boss's side, its underside and the block's side meet, the top
/// and the underside on one plane.
fn overhang() -> Solid {
    let boss =
        extrude(&Frame::at([0.0, 0.0, 10.0]), &Profile::rectangle([5.0, -5.0], [15.0, 5.0]).unwrap(), 5.0).unwrap();
    block(20.0).union(&boss).unwrap()
}

#[test]
fn an_exact_edge_picks_one_of_the_edges_two_surfaces_meet_in() {
    // The boss's side meets the z = 10 plane twice along y = 5: over the overhang, convex, where
    // the plane is the boss's underside, and over the block, concave, where it is the block's top.
    // Named by surfaces the pair picks both; named exactly, only the one its faces make. Filleted
    // R1 on its convex edges alone, each 5 long and ending flush at the saddle, the part loses
    // 20 × (1 − π/4); on its concave ones alone (two of 5 and the underside's 10) it gains as much.
    let cut = 20.0 * (1.0 - PI / 4.0);
    // Points on the faces named o.v_max, o.v_min, o.start (the underside), block.u_max, block.end.
    let (o_v_max, o_v_min, o_start) = ([10.0, 5.0, 12.0], [10.0, -5.0, 12.0], [12.5, 0.0, 10.0]);
    let (block_u_max, block_end) = ([10.0, -8.0, 5.0], [0.0, 0.0, 10.0]);
    for (edges, want) in [
        ([(o_v_max, o_start), (o_v_min, o_start), (block_u_max, block_end)], 4500.0 - cut),
        ([(block_end, o_v_max), (block_end, o_v_min), (o_start, block_u_max)], 4500.0 + cut),
    ] {
        let o = overhang();
        let exact: Vec<EdgeId> = edges.iter().flat_map(|&(a, b)| between(&o, a, b)).collect();
        let b = o.fillet(&exact, 1.0).unwrap();
        assert!(b.check().is_empty());
        assert!(watertight(&b));
        let got = b.volume();
        assert!((got - want).abs() < 1e-12 * want, "{edges:?}: volume {got} for {want}");
        assert_eq!(cylinders(&b), if want < 4500.0 { 4 } else { 3 });
    }
}
