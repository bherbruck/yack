// yack: boxes round a face and an arena, and the parameter-plane outlines and stationary points they use.
// Adapted from springrep@a03e8a1 src/measure.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from measure.rs to topo/bbox.rs (M2), so distance and mesh reach a face's box below
// them; private helpers made pub(crate) where measure still calls them. Paths only.
//! Boxes round a face (its loose box, kept on its memo, and its hull box) and round an arena, with the
//! parameter-plane outlines, winding test and stationary-point search they are found by.

use super::{Brep, FaceId, coedge_params};
use crate::{
    geom::{NurbsSurface, Patch, Surface},
    math::{Aabb, Vec2, Vec3},
    numeric::solve2,
};

/// The face's loops as parameter-plane polygons, for point classification.
pub(crate) fn uv_polygons(b: &Brep, f: FaceId) -> Vec<Vec<Vec2>> {
    uv_polygons_at(b, f, 64)
}

pub(crate) fn uv_polygons_at(b: &Brep, f: FaceId, n: usize) -> Vec<Vec<Vec2>> {
    b.face(f)
        .loops
        .iter()
        .map(|&lp| {
            let mut pts = Vec::new();
            for &co in &b.lp(lp).coedges {
                let uv = b.coedge_uvs(co, &coedge_params(b, co, n));
                pts.extend_from_slice(&uv[..uv.len() - 1]);
            }
            pts
        })
        .collect()
}

/// How many times the loops turn round `p`: the edges that cross `p`'s level to its right,
/// counted up for one rising and down for one falling. An edge owns its lower end and not its
/// upper, so a vertex on the level is counted once.
pub(crate) fn winding(polys: &[Vec<Vec2>], p: Vec2) -> i32 {
    let mut turns = 0;
    for poly in polys {
        for (k, &from) in poly.iter().enumerate() {
            let to = poly[(k + 1) % poly.len()];
            let rising = from.y <= p.y && p.y < to.y;
            let falling = to.y <= p.y && p.y < from.y;
            // `p` lies left of a rising edge, or right of a falling one, when the edge passes
            // on its right.
            let left = (to - from).cross(p - from);
            if rising && left > 0.0 {
                turns += 1;
            } else if falling && left < 0.0 {
                turns -= 1;
            }
        }
    }
    turns
}

pub(crate) fn in_face(s: &Surface, polys: &[Vec<Vec2>], uv: Vec2) -> Option<Vec2> {
    // Every whole-period shift that lands within the outline's own extent: a face can wind round
    // its surface several times (a bore's wall left between a thread's turns), its outline
    // running on across the parameter plane.
    let (mut lo, mut hi) = (
        Vec2::new(f64::INFINITY, f64::INFINITY),
        Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY),
    );
    for q in polys.iter().flatten() {
        lo = Vec2::new(lo.x.min(q.x), lo.y.min(q.y));
        hi = Vec2::new(hi.x.max(q.x), hi.y.max(q.y));
    }
    let shifts = |p: Option<f64>, x: f64, lo: f64, hi: f64| match p {
        Some(t) if lo.is_finite() => {
            let (k0, k1) = (((lo - x) / t).floor() as i64, ((hi - x) / t).ceil() as i64);
            let mut out: Vec<f64> = (k0..=k1).map(|k| k as f64 * t).collect();
            out.sort_by(|a, b| a.abs().total_cmp(&b.abs()));
            out
        }
        _ => vec![0.0],
    };
    for du in shifts(s.u_period(), uv.x, lo.x, hi.x) {
        for dv in shifts(s.v_period(), uv.y, lo.y, hi.y) {
            let q = Vec2::new(uv.x + du, uv.y + dv);
            if winding(polys, q) != 0 {
                return Some(q);
            }
        }
    }
    None
}

/// Points of the face where its coordinate along `e` is stationary and that a boundary need not
/// reach: the critical sets of planes, cylinders, cones and extrusions are lines that always meet
/// the boundary or a seam, so only the remaining surfaces have candidates.
pub(crate) fn stationary(s: &Surface, polys: &[Vec<Vec2>], e: Vec3, starts: Option<Vec<Vec2>>) -> Vec<Vec2> {
    let mut cands = Vec::new();
    match s {
        Surface::Sphere(sp) => {
            for d in [e, -e] {
                cands.push(s.project(sp.frame.origin + d * sp.radius));
            }
        }
        Surface::Torus(t) => {
            let l = t.frame.dir_to_local(e);
            let perp = l.x.hypot(l.y);
            if perp > 1e-12 {
                let u0 = l.y.atan2(l.x);
                for (u, r) in [(u0, perp), (u0 + std::f64::consts::PI, -perp)] {
                    let v = l.z.atan2(r);
                    cands.push(Vec2::new(u, v));
                    cands.push(Vec2::new(u, v + std::f64::consts::PI));
                }
            }
        }
        Surface::Revolution(_) | Surface::Ruled(_) | Surface::Nurbs(_) => {
            let (lo, hi) = uv_bounds(polys);
            let n = 12;
            let starts = starts.unwrap_or_else(|| {
                (0..=n)
                    .flat_map(|i| {
                        (0..=n).map(move |j| {
                            Vec2::new(
                                lo.x + (hi.x - lo.x) * i as f64 / n as f64,
                                lo.y + (hi.y - lo.y) * j as f64 / n as f64,
                            )
                        })
                    })
                    .collect()
            });
            for start in starts {
                let mut uv = start;
                let grad = |uv: Vec2| {
                    let (_, su, sv) = s.d1(uv.x, uv.y);
                    Vec2::new(su.dot(e), sv.dot(e))
                };
                let mut ok = false;
                for _ in 0..40 {
                    let g = grad(uv);
                    let h = 1e-6;
                    let gu = (grad(Vec2::new(uv.x + h, uv.y)) - grad(Vec2::new(uv.x - h, uv.y)))
                        / (2.0 * h);
                    let gv = (grad(Vec2::new(uv.x, uv.y + h)) - grad(Vec2::new(uv.x, uv.y - h)))
                        / (2.0 * h);
                    let Some((du, dv)) = solve2(gu.x, gv.x, gu.y, gv.y, -g.x, -g.y) else {
                        break;
                    };
                    uv = Vec2::new(uv.x + du, uv.y + dv);
                    if du.abs() + dv.abs() < 1e-13 {
                        ok = true;
                        break;
                    }
                }
                if ok {
                    cands.push(uv);
                }
            }
        }
        _ => {}
    }
    cands
        .into_iter()
        .filter_map(|uv| in_face(s, polys, uv))
        .collect()
}

/// Where to look for a B-spline face's stationary points along axis `i`: a few points in each
/// patch that the face's parameter box reaches and whose box pokes out of `bb` on that axis.
pub(crate) fn nurbs_starts(n: &NurbsSurface, polys: &[Vec<Vec2>], bb: &Aabb, i: usize) -> Vec<Vec2> {
    let (lo, hi) = uv_bounds(polys);
    let mut out = Vec::new();
    for q in reached_patches(n, polys) {
        if q.bbox.min[i] >= bb.min[i] && q.bbox.max[i] <= bb.max[i] {
            continue;
        }
        let (u0, u1, v0, v1) = if n.u_period().is_none() && n.v_period().is_none() {
            (
                q.lo.x.max(lo.x),
                q.hi.x.min(hi.x),
                q.lo.y.max(lo.y),
                q.hi.y.min(hi.y),
            )
        } else {
            (q.lo.x, q.hi.x, q.lo.y, q.hi.y)
        };
        for a in 0..3 {
            for c in 0..3 {
                out.push(Vec2::new(
                    u0 + (u1 - u0) * (a as f64 + 0.5) / 3.0,
                    v0 + (v1 - v0) * (c as f64 + 0.5) / 3.0,
                ));
            }
        }
    }
    out
}

pub(crate) fn uv_bounds(polys: &[Vec<Vec2>]) -> (Vec2, Vec2) {
    let (mut lo, mut hi) = (
        Vec2::new(f64::INFINITY, f64::INFINITY),
        Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY),
    );
    for p in polys.iter().flatten() {
        lo = Vec2::new(lo.x.min(p.x), lo.y.min(p.y));
        hi = Vec2::new(hi.x.max(p.x), hi.y.max(p.y));
    }
    (lo, hi)
}

/// The patches of `n` that the parameter box of a face's loops `polys` reaches. A periodic
/// parameter's face may sit a period away from the knot domain; every patch along it counts.
pub(crate) fn reached_patches<'a>(
    n: &'a NurbsSurface,
    polys: &[Vec<Vec2>],
) -> impl Iterator<Item = &'a Patch> + 'a {
    let (lo, hi) = uv_bounds(polys);
    patches_in(n, lo, hi)
}

/// The patches of `n` that the parameter box `lo`–`hi` reaches.
pub(crate) fn patches_in<'a>(n: &'a NurbsSurface, lo: Vec2, hi: Vec2) -> impl Iterator<Item = &'a Patch> + 'a {
    let (pu, pv) = (n.u_period().is_some(), n.v_period().is_some());
    n.patches().iter().filter(move |q| {
        (pu || (q.lo.x <= hi.x && lo.x <= q.hi.x)) && (pv || (q.lo.y <= hi.y && lo.y <= q.hi.y))
    })
}

/// A box that holds every face in the arena, cheaper than [`bbox`] and at most a little larger:
/// a B-spline face counts with the boxes of the patches its parameter box reaches.
pub fn loose_bbox(b: &Brep) -> Aabb {
    tight_or_loose_bbox(b, false)
}

/// A box holding face `f`: the smallest for every surface but a B-spline, where it may be a
/// little larger.
pub fn face_bbox(b: &Brep, f: FaceId) -> Aabb {
    face_box(b, f)
}

/// One face's share of [`loose_bbox`], from its memo.
pub(crate) fn face_box(b: &Brep, f: FaceId) -> Aabb {
    *b.memo(f).loose_box.get_or_init(|| face_loose_box(b, f))
}

/// One face's share of [`loose_bbox`]: its edges' extremes, and for a B-spline face the boxes of
/// the patches it reaches, for any other curved surface its stationary points.
pub(crate) fn face_loose_box(b: &Brep, f: FaceId) -> Aabb {
    let mut bb = Aabb::EMPTY;
    let axes = [Vec3::X, Vec3::Y, Vec3::Z];
    for &lp in &b.face(f).loops {
        for &co in &b.lp(lp).coedges {
            let edge = b.edge(b.coedge(co).edge);
            bb.add(b.vertex(edge.start).point);
            bb.add(b.vertex(edge.end).point);
            if let Some(c) = &edge.curve {
                for ax in axes {
                    for t in c.extremes(ax, edge.range.0, edge.range.1) {
                        bb.add(c.eval(t));
                    }
                }
            }
        }
    }
    let s = &b.face(f).surface;
    match s {
        Surface::Nurbs(n) => {
            // A sparse outline, its parameter box widened by a quarter each way, still holds
            // the face's parameters: a pcurve does not bulge that far between samples.
            let (lo, hi) = uv_bounds(&uv_polygons_at(b, f, 8));
            let pad = (hi - lo) * 0.25;
            for q in patches_in(n, lo - pad, hi + pad) {
                bb.union(&q.bbox);
            }
        }
        Surface::Sphere(_) | Surface::Torus(_) | Surface::Revolution(_) | Surface::Ruled(_) => {
            let polys = uv_polygons(b, f);
            for ax in axes {
                for uv in stationary(s, &polys, ax, None) {
                    bb.add(s.eval(uv.x, uv.y));
                }
            }
        }
        _ => {}
    }
    bb
}

/// A box that holds [`face_box`]'s, from control points and closed forms alone: a spline edge or
/// face counts with its control points' box, a sphere or torus with its whole extent. It costs a
/// pass over the face's edges, where [`face_box`] finds extremes, so it rules out faces far from
/// something before their [`face_box`] is worked out.
pub(crate) fn face_hull_box(b: &Brep, f: FaceId) -> Aabb {
    if let Some(bb) = b.memo(f).loose_box.get() {
        return *bb;
    }
    let mut bb = Aabb::EMPTY;
    let axes = [Vec3::X, Vec3::Y, Vec3::Z];
    for &lp in &b.face(f).loops {
        for &co in &b.lp(lp).coedges {
            let edge = b.edge(b.coedge(co).edge);
            bb.add(b.vertex(edge.start).point);
            bb.add(b.vertex(edge.end).point);
            match &edge.curve {
                None | Some(crate::geom::Curve::Line(_)) => {}
                Some(crate::geom::Curve::BSpline(s)) => {
                    s.control_points().iter().for_each(|&p| bb.add(p))
                }
                Some(crate::geom::Curve::Nurbs(n)) => {
                    n.control_points().iter().for_each(|&p| bb.add(p))
                }
                Some(c) => {
                    for ax in axes {
                        for t in c.extremes(ax, edge.range.0, edge.range.1) {
                            bb.add(c.eval(t));
                        }
                    }
                }
            }
        }
    }
    match &b.face(f).surface {
        Surface::Nurbs(n) => bb.union(&n.hull_box()),
        Surface::Sphere(s) => {
            let r = Vec3::new(s.radius, s.radius, s.radius);
            bb.add(s.frame.origin - r);
            bb.add(s.frame.origin + r);
        }
        Surface::Torus(t) => {
            let r = t.major + t.minor;
            let r = Vec3::new(r, r, r);
            bb.add(t.frame.origin - r);
            bb.add(t.frame.origin + r);
        }
        Surface::Revolution(_) | Surface::Ruled(_) => bb.union(&face_box(b, f)),
        _ => {}
    }
    // Knot insertion and the rational division round; the box gives that rounding room.
    let pad = 1e-9 * (1.0 + bb.diagonal());
    Aabb {
        min: bb.min - Vec3::new(pad, pad, pad),
        max: bb.max + Vec3::new(pad, pad, pad),
    }
}

/// The tight axis-aligned box of every face in the arena.
pub fn bbox(b: &Brep) -> Aabb {
    tight_or_loose_bbox(b, true)
}

pub(crate) fn tight_or_loose_bbox(b: &Brep, tight: bool) -> Aabb {
    if !tight {
        // Every edge of a solid bounds a face, so the faces' own loose boxes cover the arena.
        let mut bb = Aabb::EMPTY;
        for f in b.face_ids() {
            bb.union(&face_box(b, f));
        }
        for e in b.edge_ids() {
            let edge = b.edge(e);
            bb.add(b.vertex(edge.start).point);
            bb.add(b.vertex(edge.end).point);
        }
        return bb;
    }
    let mut bb = Aabb::EMPTY;
    let axes = [Vec3::X, Vec3::Y, Vec3::Z];
    for e in b.edge_ids() {
        let edge = b.edge(e);
        bb.add(b.vertex(edge.start).point);
        bb.add(b.vertex(edge.end).point);
        if let Some(c) = &edge.curve {
            for ax in axes {
                for t in c.extremes(ax, edge.range.0, edge.range.1) {
                    bb.add(c.eval(t));
                }
            }
        }
    }
    for f in b.face_ids() {
        let s = &b.face(f).surface;
        if matches!(
            s,
            Surface::Sphere(_)
                | Surface::Torus(_)
                | Surface::Revolution(_)
                | Surface::Ruled(_)
                | Surface::Nurbs(_)
        ) {
            let polys = uv_polygons(b, f);
            for (i, ax) in axes.into_iter().enumerate() {
                // A B-spline lies inside its patches' boxes: only a patch the face reaches into
                // that pokes out of the box on this axis can widen it, and only there is a
                // stationary point searched for.
                let starts = match s {
                    Surface::Nurbs(n) => {
                        let starts = nurbs_starts(n, &polys, &bb, i);
                        if starts.is_empty() {
                            continue;
                        }
                        Some(starts)
                    }
                    _ => None,
                };
                for uv in stationary(s, &polys, ax, starts) {
                    bb.add(s.eval(uv.x, uv.y));
                }
            }
        }
    }
    bb
}
