// yack: several edges of a convex polyhedron chamfered at once, each corner where all its edges are chamfered cut flat
//! Chamfers of one setback on edges of a solid of plane faces and convex edges. Each edge's chamfer
//! is the prism of [`crate::straight_chamfer`] swept past its ends, all of them made from the
//! solid as it is, so the order of the edges does not matter. Where every edge at a vertex is
//! chamfered, the chamfers there leave a point; the corner is cut flat by the plane through the
//! points where, on each face at the vertex, the two chamfers' lines on it meet: a face whose two
//! edges at the vertex make the angle `th` has that point `d / sin th` along each of them from the
//! vertex. The corner is only cut when those points lie in one plane and every other vertex of the
//! solid lies beyond it, so what is cut off is that vertex's neighbourhood alone.

use crate::{
    Error, Result, boolean,
    math::{Point3, Vec3},
    tol,
    topo::{Brep, EdgeId, FaceId, FaceName, VertexId},
};

/// `s` with each of `edges` chamfered `d` along both its faces, simultaneously, the corners where
/// all the edges at a vertex are chamfered cut flat. The chamfer of the k-th edge is named
/// `<feature>.bevel_<k>` and a corner `<feature>.corner`. Declines a solid with a face that is not a
/// plane or an edge that is not convex and straight, a vertex with only some of its edges
/// chamfered beyond one, a corner whose points are not in one plane or leave another vertex
/// behind it, and a setback that leaves a face.
pub fn chamfer_edges(s: &Brep, edges: &[EdgeId], d: f64, feature: &str) -> Result<Brep> {
    if d.is_nan() || d <= tol::LINEAR {
        return Err(Error::InvalidInput("a chamfer's distance must be positive".into()));
    }
    // One edge has no corner to cut: the straight chamfer, which also takes a concave edge.
    if let [e] = edges {
        let face = s
            .edge_uses(*e)
            .first()
            .map(|&u| s.coedge_face(u))
            .ok_or_else(|| Error::InvalidInput("a chamfered edge lies between two faces".into()))?;
        return crate::straight_chamfer::chamfer_two(s, *e, face, d, d, feature);
    }
    if s.edge_ids().any(|e| crate::topo::local::convexity(s, e) != Some(true)) {
        return Err(Error::Unsupported("a corner chamfer on a solid with an edge that is not convex".into()));
    }
    if s.face_ids().any(|f| !matches!(s.face(f).surface, crate::geom::Surface::Plane(_))) {
        return Err(Error::Unsupported("a corner chamfer on a solid with a face that is not a plane".into()));
    }
    // Each chamfer's prism, from the solid as it is, named for its edge's place in `edges`.
    let mut out = s.clone();
    for (k, &e) in edges.iter().enumerate() {
        let face = s
            .edge_uses(e)
            .first()
            .map(|&u| s.coedge_face(u))
            .ok_or_else(|| Error::InvalidInput("a chamfered edge lies between two faces".into()))?;
        let (mut prism, _) = crate::straight_chamfer::bevel_prism(s, e, face, d, d)?;
        prism.rename_faces(|n| {
            if n.feature.starts_with('\u{1}') && n.face == "a" { vec![FaceName::new(feature, format!("bevel_{k}"))] } else { Vec::new() }
        });
        out = boolean::difference(&out, &prism)?;
    }
    // The corners.
    let chamfered = |e: EdgeId| edges.contains(&e);
    for v in s.vertex_ids() {
        let at: Vec<EdgeId> = s.edge_ids().filter(|&e| s.edge(e).start == v || s.edge(e).end == v).collect();
        let mine = at.iter().filter(|&&e| chamfered(e)).count();
        if mine <= 1 {
            continue;
        }
        if mine != at.len() {
            return Err(Error::Unsupported("a vertex with some but not all of its edges chamfered".into()));
        }
        let cap = corner_cap(s, v, &at, d, feature)?;
        out = boolean::difference(&out, &cap)?;
    }
    Ok(out)
}

/// The cap of the solid beyond the corner plane at v, whose edges are at, its plane face named
/// the feature's corner.
fn corner_cap(s: &Brep, v: VertexId, at: &[EdgeId], d: f64, feature: &str) -> Result<Brep> {
    let p = s.vertex(v).point;
    let leaving = |e: EdgeId| -> Result<Vec3> {
        let x = s.edge(e);
        let other = if x.start == v { x.end } else { x.start };
        (s.vertex(other).point - p).unit().ok_or_else(|| Error::InvalidInput("an edge of no length".into()))
    };
    // On each face at the vertex, the point where its two chamfers' lines meet.
    let faces: Vec<FaceId> = {
        let mut f: Vec<FaceId> = at.iter().flat_map(|&e| s.edge_uses(e).iter().map(|&u| s.coedge_face(u)).collect::<Vec<_>>()).collect();
        f.sort();
        f.dedup();
        f
    };
    let mut points: Vec<Point3> = Vec::new();
    for f in faces {
        let on_face: Vec<EdgeId> = at.iter().copied().filter(|&e| s.edge_uses(e).iter().any(|&u| s.coedge_face(u) == f)).collect();
        let [e1, e2] = on_face[..] else {
            return Err(Error::Unsupported("a corner whose face does not have two edges at the vertex".into()));
        };
        let (u1, u2) = (leaving(e1)?, leaving(e2)?);
        let sin = (1.0 - u1.dot(u2).powi(2)).max(0.0).sqrt();
        if sin <= 1e-9 { // tol: a sine: parallel edges
            return Err(Error::Unsupported("a corner of parallel edges".into()));
        }
        points.push(p + (u1 + u2) * (d / sin));
    }
    let [q0, q1, q2, ..] = points[..] else {
        return Err(Error::Unsupported("a corner of fewer than three faces".into()));
    };
    let Some(mut n) = (q1 - q0).cross(q2 - q0).unit() else {
        return Err(Error::Unsupported("a corner whose points are on a line".into()));
    };
    if points.iter().any(|q| (*q - q0).dot(n).abs() > 10.0 * tol::LINEAR) {
        return Err(Error::Unsupported("a corner whose points are not in one plane: no flat face closes it".into()));
    }
    if n.dot(p - q0) < 0.0 {
        n = -n;
    }
    // Every other vertex strictly on the inner side: the cap is this vertex's neighbourhood alone.
    if s.vertex_ids().any(|w| w != v && n.dot(s.vertex(w).point - q0) >= -10.0 * tol::LINEAR) {
        return Err(Error::Unsupported("a corner plane that reaches past another vertex".into()));
    }
    let mut cap = boolean::half_space(s, q0, n)?;
    cap.rename_faces(|name| if name.face == FaceName::START { vec![FaceName::new(feature, "corner")] } else { Vec::new() });
    Ok(cap)
}
