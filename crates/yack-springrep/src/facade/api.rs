// yack: glue for constructors, lofts and blends (owned by w-m03-api).
//! Glue for `yack::make`, selection and blends: the arithmetic the facade may not do itself.

use crate::{
    Result,
    distance::{Nearest, Selection},
    geom::BSpline,
    math::{Aabb, Point3, Vec3},
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

/// What a pick within a tolerance found.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Pick<T> {
    /// Nothing within the tolerance.
    None,
    /// One entity, nearer than any other by at least the tolerance.
    One(T),
    /// This many entities within the tolerance whose distances differ by less than it: no telling
    /// which is meant.
    Ambiguous(usize),
}

fn pick<T: Copy>(mut found: Vec<(f64, T)>, d: f64) -> Pick<T> {
    found.sort_by(|a, b| a.0.total_cmp(&b.0));
    match found.as_slice() {
        [] => Pick::None,
        [(_, one)] => Pick::One(*one),
        [(d0, one), (d1, _), ..] => {
            if d1 - d0 >= d {
                Pick::One(*one)
            } else {
                Pick::Ambiguous(found.iter().filter(|(x, _)| x - d0 < d).count())
            }
        }
    }
}

/// The face of `b` within `d` of `p`, by the pick rule.
pub fn face_within(b: &Brep, p: Point3, d: f64) -> Result<Pick<FaceId>> {
    let mut found = Vec::new();
    for f in b.face_ids() {
        if !near(&measure::face_bbox(b, f), p, d) {
            continue;
        }
        let dist = Nearest::of(b, &Selection::face(f))?.to(p)?.distance;
        if dist <= d {
            found.push((dist, f));
        }
    }
    Ok(pick(found, d))
}

/// Whether every use of `e` is by one face: a seam, where a closed surface meets itself.
fn is_seam(b: &Brep, e: EdgeId) -> bool {
    let mut faces = b.edge_uses(e).iter().map(|&c| b.coedge_face(c));
    faces.next().is_some_and(|f| faces.all(|g| g == f))
}

/// The edge of `b` (poles aside) within `d` of `p`, by the pick rule. Seams count only when no
/// other edge is in reach: a point where a rim circle meets a cylinder's seam picks the rim.
pub fn edge_within(b: &Brep, p: Point3, d: f64) -> Result<Pick<EdgeId>> {
    let (mut found, mut seams) = (Vec::new(), Vec::new());
    for e in b.edge_ids() {
        if b.edge(e).is_degenerate() {
            continue;
        }
        let dist = Nearest::of(b, &Selection::edge(e))?.to(p)?.distance;
        if dist <= d {
            if is_seam(b, e) { seams.push((dist, e)) } else { found.push((dist, e)) }
        }
    }
    Ok(pick(if found.is_empty() { seams } else { found }, d))
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

/// The outward unit normal of face `f`'s own surface at the point of it nearest `p`.
pub fn face_normal_at(b: &Brep, f: FaceId, p: Point3) -> Option<Vec3> {
    let face = b.face(f);
    let uv = face.surface.project(p);
    let n = face.surface.normal(uv.x, uv.y).unit()?;
    Some(if face.reversed { -n } else { n })
}

/// An even `nu` × `nv` grid of points over face `f`'s parameter box (rows along u). The box
/// spans the face's outline, so on a trimmed face some points lie outside the face itself.
pub fn face_grid(b: &Brep, f: FaceId, nu: usize, nv: usize) -> Vec<Vec<Point3>> {
    let (lo, hi) = measure::face_uv_bounds(b, f);
    let s = &b.face(f).surface;
    let at = |i: usize, n: usize, a: f64, z: f64| if n <= 1 { a } else { a + (z - a) * i as f64 / (n - 1) as f64 };
    (0..nu).map(|i| (0..nv).map(|j| s.eval(at(i, nu, lo.x, hi.x), at(j, nv, lo.y, hi.y))).collect()).collect()
}

/// `n` points evenly spaced in parameter along edge `e`, ends included (empty for a pole).
pub fn edge_points(b: &Brep, e: EdgeId, n: usize) -> Vec<Point3> {
    let edge = b.edge(e);
    let Some(c) = &edge.curve else { return Vec::new() };
    let (t0, t1) = edge.range;
    let pts: Vec<Point3> = (0..n).map(|i| c.eval(if n <= 1 { t0 } else { t0 + (t1 - t0) * i as f64 / (n - 1) as f64 })).collect();
    pts
}

/// The closed C² cubic through `points` (chord-length parameters), as protocol periodic-spline
/// data: knots `u₋₃ … u_{n+3}` and the `n` control points with the first three repeated.
pub fn periodic_through(points: &[[f64; 2]]) -> Result<(Vec<f64>, Vec<[f64; 2]>)> {
    let p3: Vec<Vec3> = points.iter().map(|p| Vec3::new(p[0], p[1], 0.0)).collect();
    let s = BSpline::interpolate_periodic(&p3)?;
    Ok((s.knots().to_vec(), s.control_points().iter().map(|c| [c.x, c.y]).collect()))
}

/// The nearest point of face `f` to `p` (the face alone, wherever the rest of the solid lies).
pub fn face_closest(b: &Brep, f: FaceId, p: Point3) -> Result<Point3> {
    Ok(Nearest::of(b, &Selection::face(f))?.to(p)?.on_b)
}

/// The face nearest `p` and its distance; ties (a point nearest a shared edge) go to the face
/// whose own surface passes nearer `p`.
pub fn nearest_face(b: &Brep, p: Point3) -> Result<Option<(FaceId, f64)>> {
    let mut best: Option<(f64, f64, FaceId)> = None;
    for f in b.face_ids() {
        let d = Nearest::of(b, &Selection::face(f))?.to(p)?.distance;
        let s = &b.face(f).surface;
        let uv = s.project(p);
        let off = s.eval(uv.x, uv.y).dist(p);
        let better = match best {
            None => true,
            Some((bd, boff, _)) => d < bd - crate::tol::LINEAR || (d <= bd + crate::tol::LINEAR && off < boff),
        };
        if better {
            best = Some((d, off, f));
        }
    }
    Ok(best.map(|(d, _, f)| (f, d)))
}

/// Edge `e`'s point at curve parameter `t`, and the edge's parameter range.
pub fn edge_point(b: &Brep, e: EdgeId, t: f64) -> Option<Point3> {
    b.edge(e).curve.as_ref().map(|c| c.eval(t))
}
