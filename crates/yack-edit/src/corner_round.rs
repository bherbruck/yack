// yack: the three concave edges meeting at an inside corner of three orthogonal planes rounded to one radius
//! Three concave straight edges at a vertex, between three mutually orthogonal plane faces, rounded
//! to radius `r`. Each round is the filled strip of [`crate::straight_round`] along its edge; where
//! they meet, the corner is the cube of side `r` at the vertex less the ball of radius `r`
//! centred `r` along each edge, which each round runs into tangentially. The strips and the corner
//! block overlap only inside what the result holds, so the result is the solid joined with all four.

use crate::{
    Error, Result, boolean,
    math::{Frame, Vec2},
    profile::Profile,
    sweep::extrude,
    tol,
    topo::{Brep, EdgeId, FaceId, FaceName},
};

/// `s` with the concave edges `edges` rounded to radius `r`: the round of the k-th is named
/// `<feature>.round_<k>` and the corner `<feature>.corner`. Declines anything but exactly three
/// concave straight edges between plane faces meeting at one vertex at right angles, a vertex with
/// more faces than those three, and a radius longer than an edge.
pub fn fillet_concave_corner(s: &Brep, edges: &[EdgeId], r: f64, feature: &str) -> Result<Brep> {
    if r.is_nan() || r <= tol::LINEAR {
        return Err(Error::InvalidInput("a round's radius must be positive".into()));
    }
    let [e0, e1, e2] = edges else {
        return Err(Error::Unsupported("a corner round of other than three edges".into()));
    };
    let ends = |e: EdgeId| [s.edge(e).start, s.edge(e).end];
    let Some(v) = ends(*e0).into_iter().find(|v| ends(*e1).contains(v) && ends(*e2).contains(v)) else {
        return Err(Error::Unsupported("a corner round of edges that do not meet at one vertex".into()));
    };
    if edges.iter().any(|&e| crate::topo::local::convexity(s, e) != Some(false)) {
        return Err(Error::Unsupported("a corner round of edges that are not all concave".into()));
    }
    let faces: Vec<FaceId> = {
        let mut f: Vec<FaceId> = s.coedge_ids().filter(|&c| s.coedge_start(c) == v).map(|c| s.coedge_face(c)).collect();
        f.sort();
        f.dedup();
        f
    };
    if faces.len() != 3 {
        return Err(Error::Unsupported("a corner round at a vertex of other than three faces".into()));
    }
    let p = s.vertex(v).point;
    let mut dirs = Vec::new();
    for &e in edges {
        let other = ends(e).into_iter().find(|&w| w != v).map(|w| s.vertex(w).point);
        let Some(unit) = other.and_then(|q| (q - p).unit()) else {
            return Err(Error::Unsupported("a corner round of an edge that is not straight".into()));
        };
        if other.is_some_and(|q| q.dist(p) < r - tol::LINEAR) {
            return Err(Error::InvalidInput("a corner round whose radius is longer than an edge".into()));
        }
        dirs.push(unit);
    }
    let [u0, u1, u2] = dirs[..] else {
        return Err(Error::Unsupported("a corner round of other than three edges".into()));
    };
    if [u0.dot(u1), u1.dot(u2), u2.dot(u0)].iter().any(|c| c.abs() > 1e-9) { // tol: a cosine
        return Err(Error::Unsupported("a corner round whose edges are not at right angles".into()));
    }
    // The three strips, each from its own edge's faces.
    let mut out = s.clone();
    for (k, &e) in edges.iter().enumerate() {
        let face = s
            .edge_uses(e)
            .first()
            .map(|&u| s.coedge_face(u))
            .ok_or_else(|| Error::InvalidInput("a rounded edge lies between two faces".into()))?;
        let (mut prism, _) = crate::straight_chamfer::edge_prism(s, e, face, r, r, true)?;
        prism.rename_faces(|n| {
            if n.feature.starts_with('\u{1}') && n.face == "a" { vec![FaceName::new(feature, format!("round_{k}"))] } else { Vec::new() }
        });
        out = boolean::union(&out, &prism)?;
    }
    // The corner: the cube of side r at the vertex less the ball.
    let frame = Frame::new(p, u2, u0)?;
    let side = if frame.y.dot(u1) > 0.0 { r / 2.0 } else { -r / 2.0 };
    let mut block = extrude(&Profile::rect(Vec2::new(r / 2.0, side), r, r), &frame, r, "block")?;
    block.rename_faces(|_| Vec::new());
    let centre = p + (u0 + u1 + u2) * r;
    let mut ball = yack_build::primitive::sphere(&Frame::new(centre, u2, u0)?, r, "ball")?;
    ball.rename_faces(|_| vec![FaceName::new(feature, "corner")]);
    let corner = boolean::difference(&block, &ball)?;
    boolean::union(&out, &corner)
}

/// `s` with `edges` rounded to radius `r` where that is a straight round (one edge between two planes,
/// convex or concave) or the three concave edges of an inside corner of three orthogonal planes;
/// anything else is declined, so that no doubtful solid is built.
pub fn fillet_edges(s: &Brep, edges: &[EdgeId], r: f64, feature: &str) -> Result<Brep> {
    match edges {
        [e] => {
            let face = s
                .edge_uses(*e)
                .first()
                .map(|&u| s.coedge_face(u))
                .ok_or_else(|| Error::InvalidInput("a rounded edge lies between two faces".into()))?;
            crate::straight_round::fillet_edge(s, *e, face, r, feature)
        }
        [_, _, _] => fillet_concave_corner(s, edges, r, feature),
        _ if edges.len() == s.edge_ids().count() => crate::convex_grow::round_all_edges(s, r, feature),
        _ => Err(Error::Unsupported("a native round of this set of edges".into())),
    }
}
