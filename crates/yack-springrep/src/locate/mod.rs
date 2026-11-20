// yack: point location: a face's trimmed domain and point-in-solid classification (PLAN §3.4 T1).
//! Point location: a face's trimmed domain (`domain`) and point-in-solid classification
//! (`classify`), with the trimmed domain of a face outside any boolean. Moved out of `boolean/`
//! in M2.1 (T1) so that `check`, `measure`, `mesh`, `rays`, `distance` and `topo` need not reach
//! into the boolean.

pub(crate) mod classify;
pub(crate) mod domain;

use std::rc::Rc;

use self::domain::{Domain, Trim, UvMap};
use crate::{
    geom::{Curve, Surface},
    intersect::ssi::same_domain,
    math::Vec2,
    tol,
    topo::{Brep, CoedgeId, FaceId, PCurve, coedge_params},
};

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
        PCurve::Projected { .. } => UvMap::Projected,
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

fn analytic(s: &Surface) -> bool {
    matches!(
        s,
        Surface::Plane(_)
            | Surface::Cylinder(_)
            | Surface::Cone(_)
            | Surface::Sphere(_)
            | Surface::Torus(_)
    )
}

/// Whether two faces' surfaces are the same point set where the faces are. Analytic pairs are
/// decided in closed form by [`same_domain`]; any other pair (swept, ruled or spline surfaces in
/// different frames or parameterisations, or one of them against an analytic surface) by checking
/// that each face's parameter window lies on the other surface, with the same normal line, within
/// the tolerance.
pub(crate) fn same_domain_faces(a: &Domain, b: &Domain) -> bool {
    if same_domain(&a.surface, &b.surface) {
        return true;
    }
    // Two planes a hair off parallel are one plane over faces small enough that they stay within
    // the tolerance of each other: a part placed by a rounded pose sits on its seat that way, and
    // their meeting line, at the angle between them, would run anywhere along the band.
    let planes = matches!(
        (&a.surface, &b.surface),
        (Surface::Plane(_), Surface::Plane(_))
    );
    if analytic(&a.surface) && analytic(&b.surface) && !planes {
        return false;
    }
    let n = 6;
    [(a, b), (b, a)].iter().all(|(from, to)| {
        (0..=n).all(|i| {
            (0..=n).all(|j| {
                let u = from.lo.x + (from.hi.x - from.lo.x) * i as f64 / n as f64;
                let v = from.lo.y + (from.hi.y - from.lo.y) * j as f64 / n as f64;
                let p = from.surface.eval(u, v);
                let q = to.surface.project(p);
                to.surface.eval(q.x, q.y).dist(p) <= tol::LINEAR
                    && from
                        .surface
                        .normal(u, v)
                        .cross(to.surface.normal(q.x, q.y))
                        .norm()
                        <= tol::TANGENT
            })
        })
    })
}
