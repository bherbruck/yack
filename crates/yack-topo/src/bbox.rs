// yack: boxes round a face and an arena, and the parameter-plane outlines and stationary points they use.
// Adapted from springrep@a03e8a1 src/measure.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from measure.rs to topo/bbox.rs (M2), so distance and mesh reach a face's box below
// them; private helpers made pub(crate) where measure still calls them. Paths only; then to yack-topo (M3 W4), paths and visibility only;
// M3 hardening: the tight and loose arena boxes are separate functions, not one with a flag; an
// edge's extremes, a Newton search for a stationary point and the outline's parameter box are each
// one function; outlines walk their edges by iterators; no indexing.
//! Boxes round a face (its loose box, kept on its memo, and its hull box) and round an arena, with the
//! parameter-plane outlines, winding test and stationary-point search they are found by.

use std::f64::consts::PI;

use super::{Brep, Edge, FaceId, coedge_params};
use crate::{
    geom::{Curve, NurbsSurface, Patch, Surface},
    math::{Aabb, Vec2, Vec3},
    numeric::solve2,
};

/// The three axes a box is widened along.
const AXES: [Vec3; 3] = [Vec3::X, Vec3::Y, Vec3::Z];

/// The steps each coedge of a face's outline is sampled at, for point classification.
const OUTLINE_STEPS: usize = 64;

/// The face's loops as parameter-plane polygons, for point classification.
pub fn uv_polygons(b: &Brep, f: FaceId) -> Vec<Vec<Vec2>> {
    uv_polygons_at(b, f, OUTLINE_STEPS)
}

/// The face's loops as parameter-plane polygons, each coedge sampled at `n` steps: a polygon has
/// each coedge's points but its last, the next coedge's first.
fn uv_polygons_at(b: &Brep, f: FaceId, n: usize) -> Vec<Vec<Vec2>> {
    let outline = |coedges: &[super::CoedgeId]| {
        let mut points = Vec::new();
        for &c in coedges {
            let uvs = b.coedge_uvs(c, &coedge_params(b, c, n));
            if let Some((_, all_but_last)) = uvs.split_last() {
                points.extend_from_slice(all_but_last);
            }
        }
        points
    };
    b.face(f).loops.iter().map(|&lp| outline(&b.lp(lp).coedges)).collect()
}

/// How many times the loops turn round `p`: the edges that cross `p`'s level to its right,
/// counted up for one rising and down for one falling. An edge owns its lower end and not its
/// upper, so a vertex on the level is counted once.
pub fn winding(polys: &[Vec<Vec2>], p: Vec2) -> i32 {
    let mut turns = 0;
    for poly in polys {
        for (&from, &to) in poly.iter().zip(poly.iter().cycle().skip(1)) {
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

/// The image of `uv`, moved by whole periods, that the loops `polys` wind round; `None` where none
/// does.
pub fn in_face(s: &Surface, polys: &[Vec<Vec2>], uv: Vec2) -> Option<Vec2> {
    // Every whole-period shift that lands within the outline's own extent: a face can wind round
    // its surface several times (a bore's wall left between a thread's turns), its outline
    // running on across the parameter plane.
    let (lo, hi) = uv_bounds(polys);
    let shifts = |period: Option<f64>, x: f64, lo: f64, hi: f64| match period {
        Some(t) if lo.is_finite() => {
            let (k0, k1) = (((lo - x) / t).floor() as i64, ((hi - x) / t).ceil() as i64);
            let mut out: Vec<f64> = (k0..=k1).map(|k| k as f64 * t).collect();
            out.sort_by(|a, b| a.abs().total_cmp(&b.abs()));
            out
        }
        _ => vec![0.0],
    };
    let v_shifts = shifts(s.v_period(), uv.y, lo.y, hi.y);
    shifts(s.u_period(), uv.x, lo.x, hi.x)
        .into_iter()
        .flat_map(|du| v_shifts.iter().map(move |&dv| Vec2::new(uv.x + du, uv.y + dv)))
        .find(|&q| winding(polys, q) != 0)
}

/// The points of grid lines `n` to a side across the parameter box `lo`–`hi`, row by row in `u`.
fn grid(lo: Vec2, hi: Vec2, n: usize) -> Vec<Vec2> {
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
}

/// Where `s`'s coordinate along `e` is stationary, by Newton's method on its gradient from
/// `start`: `None` where the step converges on no point within its iterations.
fn newton_stationary(s: &Surface, e: Vec3, start: Vec2) -> Option<Vec2> {
    let gradient = |uv: Vec2| {
        let (_, su, sv) = s.d1(uv.x, uv.y);
        Vec2::new(su.dot(e), sv.dot(e))
    };
    let h = 1e-6; // tol: the finite-difference step of the gradient's derivative
    let mut uv = start;
    for _ in 0..40 {
        let g = gradient(uv);
        let gu = (gradient(Vec2::new(uv.x + h, uv.y)) - gradient(Vec2::new(uv.x - h, uv.y)))
            / (2.0 * h);
        let gv = (gradient(Vec2::new(uv.x, uv.y + h)) - gradient(Vec2::new(uv.x, uv.y - h)))
            / (2.0 * h);
        let (du, dv) = solve2(gu.x, gv.x, gu.y, gv.y, -g.x, -g.y)?;
        uv = Vec2::new(uv.x + du, uv.y + dv);
        if du.abs() + dv.abs() < 1e-13 {
            return Some(uv);
        }
    }
    None
}

/// Points of the face where its coordinate along `e` is stationary and that a boundary need not
/// reach: the critical sets of planes, cylinders, cones and extrusions are lines that always meet
/// the boundary or a seam, so only the remaining surfaces have candidates.
pub fn stationary(
    s: &Surface,
    polys: &[Vec<Vec2>],
    e: Vec3,
    starts: Option<Vec<Vec2>>,
) -> Vec<Vec2> {
    let candidates = match s {
        Surface::Sphere(sphere) => {
            [e, -e].map(|d| s.project(sphere.frame.origin + d * sphere.radius)).to_vec()
        }
        Surface::Torus(torus) => {
            let local = torus.frame.dir_to_local(e);
            let perp = local.x.hypot(local.y);
            if perp > 1e-12 {
                let u0 = local.y.atan2(local.x);
                [(u0, perp), (u0 + PI, -perp)]
                    .into_iter()
                    .flat_map(|(u, r)| {
                        let v = local.z.atan2(r);
                        [Vec2::new(u, v), Vec2::new(u, v + PI)]
                    })
                    .collect()
            } else {
                Vec::new()
            }
        }
        Surface::Revolution(_) | Surface::Ruled(_) | Surface::Nurbs(_) => {
            let starts = starts.unwrap_or_else(|| {
                let (lo, hi) = uv_bounds(polys);
                grid(lo, hi, 12)
            });
            starts.into_iter().filter_map(|start| newton_stationary(s, e, start)).collect()
        }
        _ => Vec::new(),
    };
    candidates.into_iter().filter_map(|uv| in_face(s, polys, uv)).collect()
}

/// Where to look for a B-spline face's stationary points along axis `axis`: a few points in each
/// patch that the face's parameter box reaches and whose box pokes out of `bb` on that axis.
fn nurbs_starts(n: &NurbsSurface, polys: &[Vec<Vec2>], bb: &Aabb, axis: usize) -> Vec<Vec2> {
    let (lo, hi) = uv_bounds(polys);
    let periodic = n.u_period().is_some() || n.v_period().is_some();
    let mut out = Vec::new();
    for patch in reached_patches(n, polys) {
        if patch.bbox.min[axis] >= bb.min[axis] && patch.bbox.max[axis] <= bb.max[axis] {
            continue;
        }
        let (u0, u1, v0, v1) = if periodic {
            (patch.lo.x, patch.hi.x, patch.lo.y, patch.hi.y)
        } else {
            (patch.lo.x.max(lo.x), patch.hi.x.min(hi.x), patch.lo.y.max(lo.y), patch.hi.y.min(hi.y))
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

/// The parameter box of the polygons' points: infinite the wrong way round when there are none.
fn uv_bounds(polys: &[Vec<Vec2>]) -> (Vec2, Vec2) {
    let empty = (
        Vec2::new(f64::INFINITY, f64::INFINITY),
        Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY),
    );
    polys.iter().flatten().fold(empty, |(lo, hi), p| {
        (Vec2::new(lo.x.min(p.x), lo.y.min(p.y)), Vec2::new(hi.x.max(p.x), hi.y.max(p.y)))
    })
}

/// The patches of `n` that the parameter box of a face's loops `polys` reaches. A periodic
/// parameter's face may sit a period away from the knot domain; every patch along it counts.
pub fn reached_patches<'a>(
    n: &'a NurbsSurface,
    polys: &[Vec<Vec2>],
) -> impl Iterator<Item = &'a Patch> + 'a {
    let (lo, hi) = uv_bounds(polys);
    patches_in(n, lo, hi)
}

/// The patches of `n` that the parameter box `lo`–`hi` reaches.
fn patches_in(n: &NurbsSurface, lo: Vec2, hi: Vec2) -> impl Iterator<Item = &Patch> {
    let (u_periodic, v_periodic) = (n.u_period().is_some(), n.v_period().is_some());
    n.patches().iter().filter(move |q| {
        (u_periodic || (q.lo.x <= hi.x && lo.x <= q.hi.x))
            && (v_periodic || (q.lo.y <= hi.y && lo.y <= q.hi.y))
    })
}

/// Widens `bb` to hold `edge`'s ends and its curve's extremes along each axis.
fn add_edge_extremes(bb: &mut Aabb, b: &Brep, edge: &Edge) {
    bb.add(b.vertex(edge.start).point);
    bb.add(b.vertex(edge.end).point);
    if let Some(curve) = &edge.curve {
        add_curve_extremes(bb, curve, edge.range);
    }
}

/// Widens `bb` to hold `curve`'s extremes along each axis over `range`.
fn add_curve_extremes(bb: &mut Aabb, curve: &Curve, (t0, t1): (f64, f64)) {
    for axis in AXES {
        for t in curve.extremes(axis, t0, t1) {
            bb.add(curve.eval(t));
        }
    }
}

/// The edges of face `f`'s loops, each once a coedge.
fn face_edges(b: &Brep, f: FaceId) -> impl Iterator<Item = &Edge> {
    b.face(f)
        .loops
        .iter()
        .flat_map(move |&lp| b.lp(lp).coedges.iter())
        .map(move |&c| b.edge(b.coedge(c).edge))
}

/// A box that holds every face in the arena, cheaper than [`bbox`] and at most a little larger:
/// a B-spline face counts with the boxes of the patches its parameter box reaches.
pub fn loose_bbox(b: &Brep) -> Aabb {
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
    bb
}

/// A box holding face `f`: the smallest for every surface but a B-spline, where it may be a
/// little larger.
pub fn face_bbox(b: &Brep, f: FaceId) -> Aabb {
    face_box(b, f)
}

/// One face's share of [`loose_bbox`], from its memo.
pub fn face_box(b: &Brep, f: FaceId) -> Aabb {
    *b.memo(f).loose_box.get_or_init(|| face_loose_box(b, f))
}

/// One face's share of [`loose_bbox`]: its edges' extremes, and for a B-spline face the boxes of
/// the patches it reaches, for any other curved surface its stationary points.
fn face_loose_box(b: &Brep, f: FaceId) -> Aabb {
    let mut bb = Aabb::EMPTY;
    for edge in face_edges(b, f) {
        add_edge_extremes(&mut bb, b, edge);
    }
    let s = &b.face(f).surface;
    match s {
        Surface::Nurbs(n) => {
            // A sparse outline, its parameter box widened by a quarter each way, still holds
            // the face's parameters: a pcurve does not bulge that far between samples.
            let (lo, hi) = uv_bounds(&uv_polygons_at(b, f, 8));
            let pad = (hi - lo) * 0.25;
            for patch in patches_in(n, lo - pad, hi + pad) {
                bb.union(&patch.bbox);
            }
        }
        Surface::Sphere(_) | Surface::Torus(_) | Surface::Revolution(_) | Surface::Ruled(_) => {
            let polys = uv_polygons(b, f);
            for axis in AXES {
                for uv in stationary(s, &polys, axis, None) {
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
pub fn face_hull_box(b: &Brep, f: FaceId) -> Aabb {
    if let Some(bb) = b.memo(f).loose_box.get() {
        return *bb;
    }
    let mut bb = Aabb::EMPTY;
    for edge in face_edges(b, f) {
        bb.add(b.vertex(edge.start).point);
        bb.add(b.vertex(edge.end).point);
        match &edge.curve {
            None | Some(Curve::Line(_)) => {}
            Some(Curve::BSpline(s)) => s.control_points().iter().for_each(|&p| bb.add(p)),
            Some(Curve::Nurbs(n)) => n.control_points().iter().for_each(|&p| bb.add(p)),
            Some(curve) => add_curve_extremes(&mut bb, curve, edge.range),
        }
    }
    let cube = |r: f64| Vec3::new(r, r, r);
    match &b.face(f).surface {
        Surface::Nurbs(n) => bb.union(&n.hull_box()),
        Surface::Sphere(s) => {
            bb.add(s.frame.origin - cube(s.radius));
            bb.add(s.frame.origin + cube(s.radius));
        }
        Surface::Torus(t) => {
            let r = t.major + t.minor;
            bb.add(t.frame.origin - cube(r));
            bb.add(t.frame.origin + cube(r));
        }
        Surface::Revolution(_) | Surface::Ruled(_) => bb.union(&face_box(b, f)),
        _ => {}
    }
    // Knot insertion and the rational division round; the box gives that rounding room.
    let pad = cube(1e-9 * (1.0 + bb.diagonal()));
    Aabb { min: bb.min - pad, max: bb.max + pad }
}

/// The tight axis-aligned box of every face in the arena.
pub fn bbox(b: &Brep) -> Aabb {
    let mut bb = Aabb::EMPTY;
    for e in b.edge_ids() {
        add_edge_extremes(&mut bb, b, b.edge(e));
    }
    for f in b.face_ids() {
        let s = &b.face(f).surface;
        let curved = matches!(
            s,
            Surface::Sphere(_)
                | Surface::Torus(_)
                | Surface::Revolution(_)
                | Surface::Ruled(_)
                | Surface::Nurbs(_)
        );
        if !curved {
            continue;
        }
        let polys = uv_polygons(b, f);
        for (i, axis) in AXES.into_iter().enumerate() {
            // A B-spline lies inside its patches' boxes: only a patch the face reaches into that
            // pokes out of the box on this axis can widen it, and only there is a stationary point
            // searched for.
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
            for uv in stationary(s, &polys, axis, starts) {
                bb.add(s.eval(uv.x, uv.y));
            }
        }
    }
    bb
}
