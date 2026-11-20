// yack: chamfers and fillets of an edge picked by id, measured, the faces keeping their feature names.
// Adapted from springrep@a03e8a1 tests/edges_by_id.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: springrep API replaced by the yack public API (PLAN §4.3); the edge between the faces named
// block.u_max and block.v_max is `edges_between` of the faces through points on them; springrep's centred
// `Profile::rect` is written as `Profile::rectangle(min, max)`; `name_counts` becomes `named` +
// `faces_tagged`: yack tags are feature names, so "each of the block's six names once" is six faces tagged
// "block" (which side is which is an API gap), and the offset's "g.corner"/"g.round_*" faces are told
// apart by surface kind (the corners are ball patches, the rounds cylinders).
//! Chamfers and fillets of edges picked by id: the result's faces keep the names they had.

use std::f64::consts::PI;

use yack::prelude::*;

/// The block's six faces keep the block's name ("block.start" … "block.v_max" once each).
fn own(s: &Solid) -> bool {
    s.faces_tagged("block").len() == 6
}

/// A 20 × 20 block 10 tall, and its one edge between the `u_max` (x = 10) and `v_max` (y = 10)
/// sides.
fn block_and_edge() -> (Solid, Vec<EdgeId>) {
    let square = Profile::rectangle([-10.0, -10.0], [10.0, 10.0]).unwrap();
    let b = extrude(&Frame::WORLD, &square, 10.0).unwrap().named("block");
    let edges = b.edges_between(b.face_at([10.0, 0.0, 5.0]).unwrap(), b.face_at([0.0, 10.0, 5.0]).unwrap()).unwrap();
    assert_eq!(edges.len(), 1);
    (b, edges)
}

#[test]
fn a_fillet_of_an_edge_by_id_keeps_every_face_name() {
    let (b, edges) = block_and_edge();
    let out = b.fillet(&edges, 2.0).unwrap().named("f");
    // The corner's 2 × 2 square less a quarter disc of radius 2 goes, 10 tall.
    let want = 4000.0 - 10.0 * (4.0 - PI);
    assert!((out.volume() - want).abs() < 1e-9 * want);
    assert!(own(&out));
    assert_eq!(out.faces_tagged("f").len(), 1);
    // API GAP: the six sides by their own names, and 7 names in all; needs the outline's side names
    // carried onto faces as tags.
}

#[test]
fn a_chamfer_of_an_edge_by_id_keeps_every_face_name() {
    let (b, edges) = block_and_edge();
    let out = b.chamfer(&edges, 2.0).unwrap().named("c");
    // The corner's 2 × 2 triangle goes, 10 tall.
    let want = 4000.0 - 10.0 * 2.0;
    assert!((out.volume() - want).abs() < 1e-9 * want);
    assert!(own(&out));
    assert_eq!(out.faces_tagged("c").len(), 1);
    // API GAP: the six sides by their own names, and 7 names in all; needs the outline's side names
    // carried onto faces as tags.
}

#[test]
fn a_shaped_blend_names_what_it_makes_under_its_feature() {
    let (b, edges) = block_and_edge();
    let radii = b.fillet_radii(&[(edges[0], 2.0)]).unwrap().named("r");
    assert!(own(&radii) && radii.faces_tagged("r").len() == 1);
    let conic = b.fillet_conic(&edges, 2.0, 0.5).unwrap().named("k");
    assert!(own(&conic) && conic.faces_tagged("k").len() == 1);
    let bevels = b.chamfer_distances(&[(edges[0], 2.0)]).unwrap().named("c");
    assert!(own(&bevels) && bevels.faces_tagged("c").len() == 1);
    // A block grown by 1 mm: its six faces, a round on each of twelve edges, a ball's patch at
    // each of eight corners.
    let grown = b.offset(1.0).unwrap().named("g");
    assert!(own(&grown));
    let g = grown.faces_tagged("g");
    let kind = |f: FaceId| grown.face(f).unwrap().kind();
    assert_eq!(g.iter().filter(|&&f| matches!(kind(f), SurfaceKind::Sphere { .. })).count(), 8);
    assert_eq!(g.iter().filter(|&&f| matches!(kind(f), SurfaceKind::Cylinder { .. })).count(), 12);
    // API GAP: the six sides by their own names, and 6 + 12 + 1 names in all; needs the outline's
    // side names carried onto faces as tags.
}
