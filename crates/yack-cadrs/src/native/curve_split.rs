// yack: Curves on faces: curves projected onto faces along a direction (Onshape's Projected c....
// Adapted from cadrs@eb5899ef crates/cadrs_kernel/src/native/curve_split.rs (MIT OR Apache-2.0, (c) 2026 Rouan van der Ende).
// Changes: none (verbatim).
//! Curves on faces: curves projected onto faces along a direction (Onshape's Projected curve),
//! and faces split along curves that lie on them (Onshape's Split of faces by a curve).
//!
//! A projection casts a line along the direction from points of the curve, every few degrees
//! of it, and keeps where each first meets one of the faces; the points are approximated by a
//! cubic B-spline within a hair of them (a curve leaving the faces breaks there).
//!
//! A split takes the stretch of the curve that runs across the face (its points' feet on the
//! face's surface inside the face), from where it meets the face's boundary to where it leaves
//! it: those boundary edges are split there, the stretch is a new edge (its p-curve the feet's
//! (u, v) interpolated at the curve's parameters) and the face is traced into pieces along it
//! ([`super::heal_split::trace_pieces`]).

use std::sync::Arc;

use super::classify::{InFace, face_polygons, in_face};
use super::curve::Curve;
use super::kernel::NativeKernel;
use super::math::*;
use super::nurbs::NurbsCurve;
use super::split::Arc as SArc;
use super::sweep::path::path_segments;
use super::tol;
use super::topo::{Body, Edge};
use crate::*;

fn invalid(why: &str) -> KernelError {
    KernelError::InvalidParameter(why.into())
}

/// Samples of a cut along its curve.
const CUT_SAMPLES: usize = 200;







/// The faces of `body` split along the curves (see the module docs): each face, and the pieces
/// earlier curves cut it into, cut where a curve runs across it.
pub fn split_faces_by_curves(
    k: &mut NativeKernel,
    body: BodyId,
    faces: &[FaceId],
    curves: &[PathCurve],
) -> Result<OpResult> {
    let segs = path_segments(k, curves)?;
    let mut b = (**k.get(body)?).clone();
    // The face of `body` each face is (a piece of).
    let mut from: Vec<usize> = (0..b.faces.len()).collect();
    let mut cut = false;
    for f in faces {
        let fi = f.0 as usize;
        if fi >= b.faces.len() {
            return Err(invalid("A face to split no longer exists"));
        }
        for s in &segs {
            let pieces: Vec<usize> = (0..b.faces.len()).filter(|&g| from[g] == fi).collect();
            for g in pieces {
                if let Some(out) = split_face_curve(&mut b, g, &s.c, s.t0, s.t1) {
                    from.extend(out[1..].iter().map(|_| fi));
                    cut = true;
                }
            }
        }
    }
    if !cut {
        return Err(invalid("The split curve doesn't run across the faces"));
    }
    b.fit_tolerances(1e-3);
    let history = History {
        modified: from
            .iter()
            .enumerate()
            .map(|(i, f)| {
                (
                    FaceId(i as u64),
                    InputFace {
                        body,
                        face: FaceId(*f as u64),
                    },
                )
            })
            .collect(),
        ..History::default()
    };
    let id = k.insert(b);
    Ok(OpResult {
        bodies: vec![id],
        history,
    })
}

/// Splits face `fi` of `b` along the stretch of curve `c` (over [t0, t1]) that runs across it,
/// boundary to boundary. Returns the faces it became (`fi` first); `None`, leaving `b` as it
/// was, when the curve doesn't run across the face.
pub(crate) fn split_face_curve(
    b: &mut Body,
    fi: usize,
    c: &Curve,
    t0: f64,
    t1: f64,
) -> Option<Vec<usize>> {
    let mut trial = b.clone();
    let out = split_in(&mut trial, fi, c, t0, t1)?;
    *b = trial;
    Some(out)
}

fn split_in(b: &mut Body, fi: usize, c: &Curve, t0: f64, t1: f64) -> Option<Vec<usize>> {
    let f = b.faces[fi].clone();
    let surf = f.surf.clone();
    let polys = face_polygons(b, &f);
    let size = b.face_bbox(fi).diagonal().max(tol::POINT);
    // (A curve fitted to points on the face, as a blend's rail, strays a little between them.)
    let on = 1e-4 * size;
    // 1. Which samples of the curve lie on the face.
    let at = |i: usize| t0 + (t1 - t0) * i as f64 / CUT_SAMPLES as f64;
    let mut hint: Option<(f64, f64)> = None;
    let mut inside = Vec::with_capacity(CUT_SAMPLES + 1);
    for i in 0..=CUT_SAMPLES {
        let p = c.point(at(i));
        let (u, v) = surf.project(&p, hint);
        hint = Some((u, v));
        let lies = (surf.eval(u, v) - p).norm() < on
            && in_face(&polys, &surf, P2::new(u, v), on) != InFace::Outside;
        inside.push(lies);
    }
    let first = inside.iter().position(|x| *x)?;
    let last = inside.iter().rposition(|x| *x)?;
    if last <= first || inside[first..=last].iter().any(|x| !x) {
        return None;
    }
    // Where it enters and leaves the face, between an outside sample and an inside one.
    let lies = |t: f64| -> bool {
        let p = c.point(t);
        let (u, v) = surf.project(&p, None);
        (surf.eval(u, v) - p).norm() < on
            && in_face(&polys, &surf, P2::new(u, v), on) != InFace::Outside
    };
    let edge_of = |out: f64, inn: f64| -> f64 { super::math::bisect(out, inn, |m| !lies(m)).1 };
    let ta = if first == 0 {
        t0
    } else {
        edge_of(at(first - 1), at(first))
    };
    let tb = if last == CUT_SAMPLES {
        t1
    } else {
        edge_of(at(last + 1), at(last))
    };
    // 2. The boundary edges split where it meets them (each end moved onto its edge: where
    // the curve comes nearest it).
    let mut ends = [0usize; 2];
    let mut at_ends = [ta, tb];
    let window = 2.0 * (t1 - t0) / CUT_SAMPLES as f64;
    for (k, t) in [ta, tb].into_iter().enumerate() {
        let p = c.point(t);
        let mut best: Option<(f64, usize, f64)> = None;
        for co in b.faces[fi].loops.iter().flatten() {
            let e = &b.edges[co.edge];
            if e.degenerate {
                continue;
            }
            let (lo, hi) = (e.t[0].min(e.t[1]), e.t[0].max(e.t[1]));
            let s = e.curve.closest(&p, lo, hi);
            let d = (e.point(s) - p).norm();
            if best.is_none_or(|(bd, ..)| d < bd) {
                best = Some((d, co.edge, s));
            }
        }
        let (d, e, mut s) = best?;
        if d > 1e-3 * size {
            // It ends inside the face: no cut.
            return None;
        }
        let ed = &b.edges[e];
        let (lo, hi) = (ed.t[0].min(ed.t[1]), ed.t[0].max(ed.t[1]));
        let mut t = t;
        for _ in 0..30 {
            let near = c.closest(&ed.point(s), (t - window).max(t0), (t + window).min(t1));
            let s2 = ed.curve.closest(&c.point(near), lo, hi);
            let done = (near - t).abs() < 1e-14 * (1.0 + t.abs()) && (s2 - s).abs() < 1e-14;
            t = near;
            s = s2;
            if done {
                break;
            }
        }
        at_ends[k] = t;
        ends[k] = if (ed.point(s) - b.verts[ed.v[0]].p).norm() < on {
            ed.v[0]
        } else if (ed.point(s) - b.verts[ed.v[1]].p).norm() < on {
            ed.v[1]
        } else {
            super::unify::split_edge(b, e, s)
        };
    }
    if ends[0] == ends[1] {
        return None;
    }
    let [ta, tb] = at_ends;
    // 3. The cut: the curve between, its p-curve its feet's (u, v).
    let m = 64;
    let ts: Vec<f64> = (0..=m)
        .map(|i| ta + (tb - ta) * i as f64 / m as f64)
        .collect();
    let mut hint: Option<(f64, f64)> = None;
    let uv: Vec<P3> = ts
        .iter()
        .map(|t| {
            let (u, v) = surf.project(&c.point(*t), hint);
            hint = Some((u, v));
            P3::new(u, v, 0.0)
        })
        .collect();
    let pc = Curve::Nurbs(Arc::new(NurbsCurve::interpolate(&uv, &ts, 3, None, None)));
    b.edges.push(Edge {
        curve: c.clone(),
        t: [ta, tb],
        v: ends,
        degenerate: false,
        tol: super::topo::TOL_MIN,
    });
    let cut = SArc {
        edge: b.edges.len() - 1,
        fwd: true,
        pc,
        t: [ta, tb],
        v: ends,
        cut: true,
        deg: false,
    };
    super::heal_split::trace_pieces(b, fi, vec![cut])
}
