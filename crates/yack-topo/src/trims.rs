// yack: a face's trimmed domain outside any boolean, and the trims of its coedges.
// Adapted from springrep@a03e8a1 src/boolean/mod.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved to locate/mod.rs (M2.1, T1), then to topo/trims.rs (M2, measure and locate both use
// it); paths only; then to yack-topo (M3 W4), paths and visibility only; M3 hardening: the dense
// samples come from walks::densified and are read back by stride, a trim's parameter map is one
// match, and the domain's trims take their ends by slice pattern; no indexing.
//! A face's trimmed domain outside any boolean, its trims numbered by the arena's vertices, and the
//! trim of one coedge.
#![warn(clippy::indexing_slicing)]

use std::rc::Rc;

use super::domain::{Domain, Trim, UvMap};
use super::walks::densified;
use super::{Brep, CoedgeId, FaceId, PCurve, coedge_params};
use crate::{geom::Curve, math::Vec2};

/// The dense samples a periodic face's coedge takes in each gap between the parameters asked for.
const DENSE_PER_GAP: usize = 16;

/// The steps each coedge of [`face_domain`] is sampled at.
const DOMAIN_STEPS: usize = 16;

/// A coedge's parameter-plane points at edge parameters `ts`, given in its direction; points in
/// between are sampled so periodic parameters continue across wide gaps.
pub fn coedge_uv_at(b: &Brep, c: CoedgeId, ts: &[f64]) -> Vec<Vec2> {
    if let PCurve::Affine { origin, dir } = &b.coedge(c).pcurve {
        return ts.iter().map(|&t| *origin + *dir * t).collect();
    }
    let surface = &b.face(b.coedge_face(c)).surface;
    // Only a periodic parameter needs the dense samples, to continue across a wide gap.
    if surface.u_period().is_none() && surface.v_period().is_none() {
        return b.coedge_uvs(c, ts);
    }
    let uvs = b.coedge_uvs(c, &densified(ts, DENSE_PER_GAP));
    uvs.into_iter().step_by(DENSE_PER_GAP).collect()
}

/// A trim for coedge `c` over curve parameters `t0 → t1`, at `uv0 → uv1`, between vertices `v`.
pub fn coedge_trim_with(
    b: &Brep,
    c: CoedgeId,
    piece: usize,
    curve: Option<Rc<Curve>>,
    (t0, t1): (f64, f64),
    (uv0, uv1): (Vec2, Vec2),
    (v0, v1): (usize, usize),
) -> Trim {
    let coedge = b.coedge(c);
    let edge = b.edge(coedge.edge);
    let map = match (&coedge.pcurve, &edge.curve) {
        (PCurve::Affine { origin, dir }, _) => UvMap::Affine { origin: *origin, dir: *dir },
        // A pole's parameter-plane image runs on from where it starts, at a single point.
        (PCurve::Projected { .. }, None) => UvMap::Affine { origin: uv0, dir: Vec2::ZERO },
        (PCurve::Projected { .. }, Some(_)) => UvMap::Projected,
        (PCurve::Fitted { curve }, _) => UvMap::Fitted(Rc::new((**curve).clone())),
    };
    Trim {
        piece,
        forward: !coedge.reversed,
        curve,
        pole: b.vertex(edge.start).point,
        t0,
        t1,
        map,
        uv0,
        uv1,
        v0,
        v1,
    }
}

/// Face `f`'s trimmed domain outside any boolean: its trims numbered by the arena's vertices.
pub fn face_domain(b: &Brep, f: FaceId) -> Domain {
    let face = b.face(f);
    let trim = |c: CoedgeId| {
        let ts = coedge_params(b, c, DOMAIN_STEPS);
        let uvs = coedge_uv_at(b, c, &ts);
        // Both hold `DOMAIN_STEPS + 1` samples.
        let (&[t0, .., t1], &[uv0, .., uv1]) = (ts.as_slice(), uvs.as_slice()) else {
            return None;
        };
        let curve = b.edge(b.coedge(c).edge).curve.clone().map(Rc::new);
        let vertices = (b.coedge_start(c).index(), b.coedge_end(c).index());
        Some(coedge_trim_with(b, c, usize::MAX, curve, (t0, t1), (uv0, uv1), vertices))
    };
    let loops = face
        .loops
        .iter()
        .map(|&lp| b.lp(lp).coedges.iter().filter_map(|&c| trim(c)).collect())
        .collect();
    Domain::new(face.surface.clone(), face.reversed, loops)
}
