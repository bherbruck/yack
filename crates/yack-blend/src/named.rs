// yack: a blend's faces and edges picked by the names faces carry.
// Adapted from springrep@a03e8a1 src/pick.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from pick.rs to blend/named.rs (M2.5, T8), pick being dissolved; paths only; then to yack-blend (M3 W9a), paths and visibility only.
//! A blend's faces and edges picked by the names faces carry.

use std::collections::BTreeSet;

use super::{has_name, picked_edges};
use crate::{
    Result,
    intersect::ssi::same_domain,
    topo::{Brep, EdgeId, EdgeName, FaceId},
};

/// The faces `name` stands for when an operation picks faces by name: those carrying it, and every
/// face on the surface of one. A name picks by surface because an edge is defined by the surfaces
/// its two faces lie on: where a later feature's face runs on along a named face's surface (a pad
/// flush with a block's side), the edge between them runs on too. Empty when no face carries it.
pub fn faces_named(body: &Brep, name: &str) -> BTreeSet<FaceId> {
    let carriers: Vec<FaceId> = body
        .face_ids()
        .filter(|&f| has_name(body, f, name))
        .collect();
    body.face_ids()
        .filter(|&f| {
            carriers.iter().any(|&c| {
                c == f || same_domain(&body.face(c).surface, &body.face(f).surface)
            })
        })
        .collect()
}

/// The edges `pair` names, and every edge those run on into tangentially, as a blend of the pair
/// takes them. An error when the pair names no edge.
pub fn edges_named(body: &Brep, pair: &EdgeName) -> Result<Vec<EdgeId>> {
    picked_edges(body, pair)
}
