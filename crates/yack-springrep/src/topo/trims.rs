// yack: a face's trimmed domain outside any boolean, and the trims of its coedges.
// Adapted from springrep@a03e8a1 src/boolean/mod.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved to locate/mod.rs (M2.1, T1), then to topo/trims.rs (M2, measure and locate both use
// it); paths only.
//! A face's trimmed domain outside any boolean, its trims numbered by the arena's vertices, and the
//! trim of one coedge.

use std::rc::Rc;

use super::domain::{Domain, Trim, UvMap};
use super::{Brep, CoedgeId, FaceId, PCurve, coedge_params};
use crate::{geom::Curve, math::Vec2};

/// A coedge's parameter-plane points at edge parameters `ts`, given in its direction; points in
/// between are sampled so periodic parameters continue across wide gaps.
pub(crate) fn coedge_uv_at(b: &Brep, c: CoedgeId, ts: &[f64]) -> Vec<Vec2> {
    if let PCurve::Affine { origin, dir } = &b.coedge(c).pcurve {
        return ts.iter().map(|&t| *origin + *dir * t).collect();
    }
    let s = &b.face(b.coedge_face(c)).surface;
    // Only a periodic parameter needs the dense samples, to continue across a wide gap.
    if s.u_period().is_none() && s.v_period().is_none() {
        return b.coedge_uvs(c, ts);
    }
    let per = 16;
    let mut dense = Vec::new();
    for w in ts.windows(2) {
        for k in 0..per {
            dense.push(w[0] + (w[1] - w[0]) * k as f64 / per as f64);
        }
    }
    dense.push(ts[ts.len() - 1]);
    let uvs = b.coedge_uvs(c, &dense);
    (0..ts.len()).map(|i| uvs[i * per]).collect()
}

/// A trim for coedge `c` over curve parameters `t0 → t1`, at `uv0 → uv1`, between vertices `v`.
pub(crate) fn coedge_trim_with(
    b: &Brep,
    c: CoedgeId,
    piece: usize,
    curve: Option<Rc<Curve>>,
    (t0, t1): (f64, f64),
    (uv0, uv1): (Vec2, Vec2),
    (v0, v1): (usize, usize),
) -> Trim {
    let co = b.coedge(c);
    let edge = b.edge(co.edge);
    let map = match &co.pcurve {
        PCurve::Affine { origin, dir } => UvMap::Affine {
            origin: *origin,
            dir: *dir,
        },
        PCurve::Projected { .. } | PCurve::Fitted { .. } => UvMap::Projected,
    };
    Trim {
        piece,
        forward: !co.reversed,
        curve,
        pole: b.vertex(edge.start).point,
        t0,
        t1,
        map: if edge.curve.is_none() {
            match map {
                UvMap::Projected => UvMap::Affine {
                    origin: uv0,
                    dir: Vec2::ZERO,
                },
                m => m,
            }
        } else {
            map
        },
        uv0,
        uv1,
        v0,
        v1,
    }
}

/// Face `f`'s trimmed domain outside any boolean: its trims numbered by the arena's vertices.
pub(crate) fn face_domain(b: &Brep, f: FaceId) -> Domain {
    let face = b.face(f);
    let loops = face
        .loops
        .iter()
        .map(|&lp| {
            b.lp(lp)
                .coedges
                .iter()
                .map(|&c| {
                    let ts = coedge_params(b, c, 16);
                    let uvs = coedge_uv_at(b, c, &ts);
                    let curve = b.edge(b.coedge(c).edge).curve.clone().map(Rc::new);
                    let v = (b.coedge_start(c).index(), b.coedge_end(c).index());
                    let (t, uv) = ((ts[0], ts[16]), (uvs[0], uvs[16]));
                    coedge_trim_with(b, c, usize::MAX, curve, t, uv, v)
                })
                .collect()
        })
        .collect();
    Domain::new(face.surface.clone(), face.reversed, loops)
}
