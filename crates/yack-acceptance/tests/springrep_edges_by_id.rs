// yack: chamfers and fillets of an edge picked by id, measured, every face keeping the name it had.
// Adapted from springrep@a03e8a1 tests/edges_by_id.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: springrep API replaced by the yack public API (PLAN §4.3); springrep's face name
// `feature.face` is the pair (tag, role) a face carries in yack (`named(feature)` + `FaceRef::role`):
// `start`/`end` are `Role::Start`/`Role::End`, a rectangle's v_min, u_max, v_max, u_min are
// `Role::Side("s0".."s3")`, `round_<n>`/`bevel_<n>` are `Role::Round(n)`/`Role::Bevel(n)`, and a shaped
// blend's bare `round`/`bevel` is its tag with no role; `name_counts` counts faces per (tag, role); the
// edge between block.u_max and block.v_max is `edges_between` of those faces; springrep's centred
// `Profile::rect` is written as `Profile::rectangle(min, max)`; the offset's names are an API gap
// (`offset` makes faces `named` cannot reach).
//! Chamfers and fillets of edges picked by id: the result's faces keep the names they had.
#![allow(clippy::unwrap_used)] // helper fns outside #[test] unwrap test geometry

use std::collections::BTreeMap;
use std::f64::consts::PI;

use yack::prelude::*;

type Name = (Tag, Option<Role>);

/// springrep's `name_counts`: how many faces carry each name.
fn name_counts(s: &Solid) -> BTreeMap<Name, usize> {
    let mut names = BTreeMap::new();
    for f in s.faces() {
        let face = s.face(f).unwrap();
        for t in face.tags() {
            *names.entry((t.clone(), face.role(t.clone()))).or_insert(0) += 1;
        }
    }
    names
}

fn name(tag: &str, role: Option<Role>) -> Name {
    (tag.into(), role)
}

fn side(n: u32) -> Role {
    Role::Side(format!("s{n}").into())
}

/// The block's own names: start, end, u_min, u_max, v_min, v_max.
fn own_names() -> [Name; 6] {
    [
        name("block", Some(Role::Start)),
        name("block", Some(Role::End)),
        name("block", Some(side(3))),
        name("block", Some(side(1))),
        name("block", Some(side(0))),
        name("block", Some(side(2))),
    ]
}

/// A 20 × 20 block 10 tall, and its one edge between the `u_max` and `v_max` sides.
fn block_and_edge() -> (Solid, Vec<EdgeId>) {
    let square = Profile::rectangle([-10.0, -10.0], [10.0, 10.0]).unwrap();
    let b = extrude(&Frame::WORLD, &square, 10.0).unwrap().named("block");
    let edges =
        b.edges_between(b.face_by("block", side(1)).unwrap(), b.face_by("block", side(2)).unwrap()).unwrap();
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
    let names = name_counts(&out);
    for n in own_names() {
        assert_eq!(names.get(&n), Some(&1), "{n:?} in {names:?}");
    }
    assert_eq!(names.get(&name("f", Some(Role::Round(0)))), Some(&1), "{names:?}");
    assert_eq!(names.len(), 7, "{names:?}");
}

#[test]
fn a_chamfer_of_an_edge_by_id_keeps_every_face_name() {
    let (b, edges) = block_and_edge();
    let out = b.chamfer(&edges, 2.0).unwrap().named("c");
    // The corner's 2 × 2 triangle goes, 10 tall.
    let want = 4000.0 - 10.0 * 2.0;
    assert!((out.volume() - want).abs() < 1e-9 * want);
    let names = name_counts(&out);
    for n in own_names() {
        assert_eq!(names.get(&n), Some(&1), "{n:?} in {names:?}");
    }
    assert_eq!(names.get(&name("c", Some(Role::Bevel(0)))), Some(&1), "{names:?}");
    assert_eq!(names.len(), 7, "{names:?}");
}

#[test]
fn a_shaped_blend_names_what_it_makes_under_its_feature() {
    let (b, edges) = block_and_edge();
    let own = |names: &BTreeMap<Name, usize>| own_names().iter().all(|n| names.get(n) == Some(&1));
    let radii = name_counts(&b.fillet_radii(&[(edges[0], 2.0)]).unwrap().named("r"));
    assert!(own(&radii) && radii.get(&name("r", None)) == Some(&1) && radii.len() == 7, "{radii:?}");
    let conic = name_counts(&b.fillet_conic(&edges, 2.0, 0.5).unwrap().named("k"));
    assert!(own(&conic) && conic.get(&name("k", None)) == Some(&1) && conic.len() == 7, "{conic:?}");
    let bevels = name_counts(&b.chamfer_distances(&[(edges[0], 2.0)]).unwrap().named("c"));
    assert!(own(&bevels) && bevels.get(&name("c", None)) == Some(&1) && bevels.len() == 7, "{bevels:?}");
    // A block grown by 1 mm: its six faces, a round on each of twelve edges, a ball's patch at
    // each of eight corners.
    let grown = name_counts(&b.offset(1.0).unwrap());
    assert!(own(&grown), "{grown:?}");
    // API GAP: "g.corner" on 8 faces, 12 distinct "g.round_*" names and 6 + 12 + 1 names in all;
    // `offset` makes its faces under no operation `named` can tag, so they cannot be called "g".
}
