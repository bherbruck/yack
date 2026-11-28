// yack: a straight edge between two planes rounded by a circular section, convex (cut) or concave (filled)
//! The round's section is the corner, the two feet `r / tan(phi / 2)` along the faces and the arc
//! between them (see [`crate::straight_chamfer`]'s prism, which it shares); swept along the edge it
//! is cut from a convex solid or added to a concave edge.

use crate::{
    Result, boolean,
    topo::{Brep, EdgeId, FaceId, FaceName},
};

/// `s` with the straight edge `e`, between two planes, one of them `face`, rounded to radius `r`;
/// the round is named `<feature>.round_0`. Declines what [`crate::straight_chamfer::chamfer_two`]
/// declines, and a radius whose feet leave a face.
pub fn fillet_edge(s: &Brep, e: EdgeId, face: FaceId, r: f64, feature: &str) -> Result<Brep> {
    let (prism, convex) = crate::straight_chamfer::edge_prism(s, e, face, r, r, true)?;
    let mut done = if convex { boolean::difference(s, &prism)? } else { boolean::union(s, &prism)? };
    done.rename_faces(|n| match n.feature.strip_prefix('\u{1}') {
        Some(_) => vec![FaceName::new(feature, "round_0")],
        None => vec![n.clone()],
    });
    Ok(done)
}
