// yack: Chamfers and fillets of edges picked by id: the result's faces keep the names they had.
// Adapted from springrep@a03e8a1 tests/edges_by_id.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: crate paths springrep:: -> yack_springrep::; this header added. Moved to crates/yack/tests at W11 (springrep deleted): paths to the crates the items live in.
//! Chamfers and fillets of edges picked by id: the result's faces keep the names they had.
// W11: springrep's own kernel tests, moved here when springrep went (ruling (b)). They build with the
// kernel crates directly and compute their expected values, so the facade's maths ban and the
// workspace lints' test-only exemptions (which reach only #[test] fns) do not fit them.
// TODO(api-owner, hardening, D26): rewrite these tests to the lints, test by test, and drop this allow.
#![allow(clippy::disallowed_methods, clippy::disallowed_types, clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::indexing_slicing)]

use std::f64::consts::PI;

use yack_blend::EdgeName;
use yack_blend::chamfer_distances;
use yack_blend::chamfer_edges;
use yack_blend::fillet_conic;
use yack_blend::fillet_edges;
use yack_blend::fillet_radii;
use yack_edit as edit;
use yack_math::kernel::Frame;
use yack_math::kernel::Vec2;
use yack_measure as measure;
use yack_blend::edges_named;
use yack_profile::Profile;
use yack_build::sweep::extrude;
use yack_topo::Brep;
use yack_topo::EdgeId;

/// A 20 × 20 block 10 tall, and its one edge between the `u_max` and `v_max` sides.
fn block_and_edge() -> (Brep, Vec<EdgeId>) {
    let square = Profile::rect(Vec2::ZERO, 20.0, 20.0);
    let b = extrude(&square, &Frame::WORLD, 10.0, "block").unwrap();
    let pair = EdgeName {
        a: "block.u_max",
        b: "block.v_max",
        exact: false,
    };
    let edges = edges_named(&b, &pair).unwrap();
    assert_eq!(edges.len(), 1);
    (b, edges)
}

const OWN: [&str; 6] = [
    "block.start",
    "block.end",
    "block.u_min",
    "block.u_max",
    "block.v_min",
    "block.v_max",
];

#[test]
fn a_fillet_of_an_edge_by_id_keeps_every_face_name() {
    let (b, edges) = block_and_edge();
    let out = fillet_edges(&b, &edges, 2.0, "f").unwrap();
    // The corner's 2 × 2 square less a quarter disc of radius 2 goes, 10 tall.
    let want = 4000.0 - 10.0 * (4.0 - PI);
    assert!((measure::volume(&out) - want).abs() < 1e-9 * want);
    let names = out.name_counts();
    for n in OWN {
        assert_eq!(names.get(n), Some(&1), "{n} in {names:?}");
    }
    assert_eq!(names.get("f.round_0"), Some(&1), "{names:?}");
    assert_eq!(names.len(), 7, "{names:?}");
}

#[test]
fn a_chamfer_of_an_edge_by_id_keeps_every_face_name() {
    let (b, edges) = block_and_edge();
    let out = chamfer_edges(&b, &edges, 2.0, "c").unwrap();
    // The corner's 2 × 2 triangle goes, 10 tall.
    let want = 4000.0 - 10.0 * 2.0;
    assert!((measure::volume(&out) - want).abs() < 1e-9 * want);
    let names = out.name_counts();
    for n in OWN {
        assert_eq!(names.get(n), Some(&1), "{n} in {names:?}");
    }
    assert_eq!(names.get("c.bevel_0"), Some(&1), "{names:?}");
    assert_eq!(names.len(), 7, "{names:?}");
}

#[test]
fn a_shaped_blend_names_what_it_makes_under_its_feature() {
    let (b, edges) = block_and_edge();
    let own = |names: &std::collections::BTreeMap<String, usize>| {
        OWN.iter().all(|n| names.get(*n) == Some(&1))
    };
    let radii = fillet_radii(&b, &[(edges[0], 2.0)], "r")
        .unwrap()
        .name_counts();
    assert!(
        own(&radii) && radii.get("r.round") == Some(&1) && radii.len() == 7,
        "{radii:?}"
    );
    let conic = fillet_conic(&b, &edges, 2.0, 0.5, "k")
        .unwrap()
        .name_counts();
    assert!(
        own(&conic) && conic.get("k.round") == Some(&1) && conic.len() == 7,
        "{conic:?}"
    );
    let bevels = chamfer_distances(&b, &[(edges[0], 2.0)], "c")
        .unwrap()
        .name_counts();
    assert!(
        own(&bevels) && bevels.get("c.bevel") == Some(&1) && bevels.len() == 7,
        "{bevels:?}"
    );
    // A block grown by 1 mm: its six faces, a round on each of twelve edges, a ball's patch at
    // each of eight corners. Each round also names the block edge it grows from (`round_e<k>`)
    // and each corner patch the block vertex (`corner_v<k>`), for History's origins (ab8c4a8).
    let grown = edit::offset(&b, 1.0, "g").unwrap().name_counts();
    assert!(
        own(&grown) && grown.get("g.corner") == Some(&8),
        "{grown:?}"
    );
    let starting = |prefix: &str| grown.keys().filter(|n| n.starts_with(prefix)).count();
    assert_eq!(starting("g.round_") - starting("g.round_e"), 12, "{grown:?}");
    assert_eq!(starting("g.round_e"), 12, "{grown:?}");
    assert_eq!(starting("g.corner_v"), 8, "{grown:?}");
    assert_eq!(grown.len(), 6 + 12 + 12 + 1 + 8, "{grown:?}");
}
