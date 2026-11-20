// yack: glue for constructors, lofts and blends (owned by w-m03-api).
//! Glue for `yack::make`, selection and blends: the arithmetic the facade may not do itself.

use crate::{
    Result,
    distance::{Nearest, Selection},
    math::{Aabb, Point3},
    measure,
    topo::{Brep, EdgeId, FaceId},
};

/// Whether `p` lies within `d` of the box.
fn near(b: &Aabb, p: Point3, d: f64) -> bool {
    p.x >= b.min.x - d
        && p.y >= b.min.y - d
        && p.z >= b.min.z - d
        && p.x <= b.max.x + d
        && p.y <= b.max.y + d
        && p.z <= b.max.z + d
}

/// Every face of `b` passing within `d` of `p`.
pub fn faces_within(b: &Brep, p: Point3, d: f64) -> Result<Vec<FaceId>> {
    let mut out = Vec::new();
    for f in b.face_ids() {
        if !near(&measure::face_bbox(b, f), p, d) {
            continue;
        }
        if Nearest::of(b, &Selection::face(f))?.to(p)?.distance <= d {
            out.push(f);
        }
    }
    Ok(out)
}

/// Every edge of `b` (poles aside) passing within `d` of `p`.
pub fn edges_within(b: &Brep, p: Point3, d: f64) -> Result<Vec<EdgeId>> {
    let mut out = Vec::new();
    for e in b.edge_ids() {
        if b.edge(e).is_degenerate() {
            continue;
        }
        if Nearest::of(b, &Selection::edge(e))?.to(p)?.distance <= d {
            out.push(e);
        }
    }
    Ok(out)
}

/// How the two faces meet along `e`, at its midpoint: `Some(true)` convex (material in a wedge
/// narrower than a half turn), `Some(false)` concave, `None` where they meet tangentially, or
/// the edge is not used by exactly two faces.
pub fn convexity(b: &Brep, e: EdgeId) -> Option<bool> {
    let edge = b.edge(e);
    let c = edge.curve.as_ref()?;
    let [cf, cg] = b.edge_uses(e) else { return None };
    let (f, g) = (b.coedge_face(*cf), b.coedge_face(*cg));
    if f == g {
        return None;
    }
    let outward = |f: FaceId, p: Point3| {
        let s = &b.face(f).surface;
        let uv = s.project(p);
        let n = s.normal(uv.x, uv.y);
        if b.face(f).reversed { -n } else { n }
    };
    let (p, dp) = c.d1(0.5 * (edge.range.0 + edge.range.1));
    let (n, m) = (outward(f, p), outward(g, p));
    if n.cross(m).norm() <= crate::tol::TANGENT {
        return None;
    }
    let t = if b.coedge(*cf).reversed { -dp } else { dp };
    Some(m.dot(n.cross(t)) < 0.0)
}
