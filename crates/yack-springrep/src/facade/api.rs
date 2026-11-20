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

// ---- guards: shapes springrep would build wrong, which its rung must decline (M1.10) ----

/// Whether `a` and `b` have the same face count, volume and area (to the linear tolerance,
/// relative): an edit that "succeeded" without changing the solid.
pub fn same_shape(a: &Brep, b: &Brep) -> bool {
    let close = |x: f64, y: f64| (x - y).abs() <= crate::tol::LINEAR * x.abs().max(y.abs()).max(1.0);
    a.face_count() == b.face_count()
        && close(measure::volume(a), measure::volume(b))
        && close(measure::area(a), measure::area(b))
}

/// Whether moving the faces of `b` (all, or those in `only`) `by` mm, into the material when
/// `into` (a shell's walls) else out of it (an offset growing; a negative `by` reverses either),
/// collapses a round face: a convex cylinder, sphere or torus tube of radius at most the inward
/// move, or a concave one (a hole) of radius at most the outward move.
pub fn radius_collapses(b: &Brep, only: Option<&[FaceId]>, by: f64, into: bool) -> bool {
    let inward = if into { by } else { -by };
    use crate::geom::Surface;
    b.face_ids().filter(|f| only.is_none_or(|o| o.contains(f))).any(|f| {
        let face = b.face(f);
        let r = match &face.surface {
            Surface::Cylinder(c) => c.radius,
            Surface::Sphere(s) => s.radius,
            Surface::Torus(t) => t.minor,
            _ => return false,
        };
        let convex = !face.reversed;
        if convex { inward >= r } else { -inward >= r }
    })
}

/// Whether an inward shell of `thickness` makes two flat walls cross: two planar faces (not
/// `open`) facing each other across material, nearer than twice the thickness, whose boxes
/// overlap across the gap.
pub fn walls_cross(b: &Brep, open: &[FaceId], thickness: f64) -> bool {
    use crate::geom::Surface;
    let planes: Vec<(FaceId, Point3, Vec3)> = b
        .face_ids()
        .filter(|f| !open.contains(f))
        .filter_map(|f| {
            let face = b.face(f);
            let Surface::Plane(p) = &face.surface else { return None };
            let n = if face.reversed { -p.frame.z } else { p.frame.z };
            Some((f, p.frame.origin, n))
        })
        .collect();
    let gap = 2.0 * thickness;
    planes.iter().enumerate().any(|(i, &(f, o, n))| {
        planes.iter().skip(i + 1).any(|&(g, q, m)| {
            if n.dot(m) > -1.0 + crate::tol::TANGENT {
                return false;
            }
            let d = -(q - o).dot(n);
            if d <= 0.0 || d >= gap {
                return false;
            }
            let (bf, bg) = (measure::face_bbox(b, f), measure::face_bbox(b, g));
            let pad = d + crate::tol::LINEAR;
            bf.min.x <= bg.max.x + pad
                && bg.min.x <= bf.max.x + pad
                && bf.min.y <= bg.max.y + pad
                && bg.min.y <= bf.max.y + pad
                && bf.min.z <= bg.max.z + pad
                && bg.min.z <= bf.max.z + pad
        })
    })
}

/// Whether the plane through `origin` square to `normal` passes through `b`'s interior extent
/// (more than the linear tolerance into it on both sides), so a split must leave two parts.
pub fn plane_cuts(b: &Brep, origin: Point3, normal: Vec3) -> bool {
    let Some(n) = normal.unit() else { return false };
    let (lo, hi) = measure::extent(b, n);
    let at = origin.dot(n);
    lo < at - crate::tol::LINEAR && hi > at + crate::tol::LINEAR
}

/// Whether every rim of the sheet `faces` meets its neighbour square to the sheet (the sheet's
/// normal lies in the neighbour face at the rim's midpoint), as springrep's thicken assumes when
/// it sweeps the rims along the neighbours.
pub fn rims_square(b: &Brep, faces: &[FaceId]) -> bool {
    for &f in faces {
        for &l in &b.face(f).loops {
            for &c in &b.lp(l).coedges {
                let e = b.coedge(c).edge;
                let edge = b.edge(e);
                let Some(curve) = &edge.curve else { continue };
                let p = curve.eval(0.5 * (edge.range.0 + edge.range.1));
                let Some(n) = face_normal_at(b, f, p) else { return false };
                for &u in b.edge_uses(e) {
                    let g = b.coedge_face(u);
                    if g == f || faces.contains(&g) {
                        continue;
                    }
                    let Some(m) = face_normal_at(b, g, p) else { return false };
                    if n.dot(m).abs() > crate::tol::TANGENT {
                        return false;
                    }
                }
            }
        }
    }
    true
}

/// Whether the bounding boxes of `a` and `b` overlap with positive volume (by more than the
/// linear tolerance on every axis).
pub fn boxes_overlap(a: &Brep, b: &Brep) -> bool {
    let (p, q) = (measure::bbox(a), measure::bbox(b));
    let t = crate::tol::LINEAR;
    p.min.x + t < q.max.x && q.min.x + t < p.max.x && p.min.y + t < q.max.y && q.min.y + t < p.max.y && p.min.z + t < q.max.z && q.min.z + t < p.max.z
}

/// Whether an open face of a shell meets a concave edge (springrep's sharp shell of an L block open
/// at its step face comes out wrong, inward and outward).
pub fn open_at_concave_edge(b: &Brep, open: &[FaceId]) -> bool {
    open.iter().any(|&f| {
        b.face(f).loops.iter().any(|&l| b.lp(l).coedges.iter().any(|&c| convexity(b, b.coedge(c).edge) == Some(false)))
    })
}

/// Whether the blend `feature` made in `out` strays from the given edges: some connected group of
/// its blend faces (faces of `feature` whose part starts `part`, joined where they share an edge)
/// comes no nearer than `reach` to any of the given edges' midpoints `near`. A fillet of one rim
/// that also rounded another rim makes such a group (brep-bench edit/half-torus-outer-rim-round-deleted).
pub fn blend_strays(out: &Brep, feature: &str, part: &str, near: &[Point3], reach: f64) -> bool {
    if near.is_empty() {
        return false;
    }
    let blend: Vec<FaceId> = out
        .face_ids()
        .filter(|&f| out.face(f).names.iter().any(|n| n.feature == feature && n.face.starts_with(part)))
        .collect();
    let edges_of = |f: FaceId| -> std::collections::BTreeSet<EdgeId> {
        out.face(f).loops.iter().flat_map(|&l| out.lp(l).coedges.iter().map(|&c| out.coedge(c).edge)).collect()
    };
    let sets: Vec<std::collections::BTreeSet<EdgeId>> = blend.iter().map(|&f| edges_of(f)).collect();
    let mut group: Vec<usize> = (0..blend.len()).collect();
    fn root(g: &mut [usize], i: usize) -> usize {
        let mut r = i;
        while g.get(r).copied().unwrap_or(r) != r {
            r = g.get(r).copied().unwrap_or(r);
        }
        r
    }
    for i in 0..blend.len() {
        for j in (i + 1)..blend.len() {
            if let (Some(a), Some(b)) = (sets.get(i), sets.get(j))
                && !a.is_disjoint(b)
            {
                let (ri, rj) = (root(&mut group, i), root(&mut group, j));
                if let Some(slot) = group.get_mut(ri) {
                    *slot = rj;
                }
            }
        }
    }
    let mut best: std::collections::BTreeMap<usize, f64> = std::collections::BTreeMap::new();
    for (i, &f) in blend.iter().enumerate() {
        let d = near
            .iter()
            .filter_map(|&p| Nearest::of(out, &Selection::face(f)).ok()?.to(p).ok().map(|c| c.distance))
            .fold(f64::INFINITY, f64::min);
        let r = root(&mut group, i);
        let e = best.entry(r).or_insert(f64::INFINITY);
        *e = e.min(d);
    }
    best.values().any(|&d| d > reach)
}

/// Whether springrep's refusal `detail` from yack op `op` says the input breaks a stated
/// requirement of the operation (yack's §4.3 docs, which follow brep-conformance doc/api.md);
/// any other springrep refusal is a limit of springrep, not of the request. Each entry names the
/// requirement it stands for.
pub fn breaks_requirement(op: &str, detail: &str) -> bool {
    const BLENDS: &[&str] = &[
        "fillet", "fillet_radii", "fillet_variable", "fillet_conic", "fillet_g2", "fillet_setback", "fillet_setback_g2",
        "fillet_faces", "full_round", "chamfer", "chamfer_two", "chamfer_distances",
    ];
    let is = |ops: &[&str]| ops.contains(&op);
    // sizes, radii, heights and blend sizes must be positive (block, cylinder, sphere, cone, every blend)
    detail.contains("must be positive")
        // a radius or distance must fit the faces it lies on (every fillet and chamfer)
        || (is(BLENDS)
            && (detail.contains("takes a whole face")
                || detail.contains("runs past the end of an edge")
                || detail.contains("does not fit on its faces")
                || detail.contains("the faces are parallel")))
        // a cone needs a positive height and a radius at one end at least
        || (op == "cone" && detail.contains("needs a positive height"))
        // an extrusion's height must be positive
        || (op == "extrude" && detail.contains("must be nonzero"))
        // loft_through's heights must rise
        || (op == "loft_through" && detail.contains("do not come in order"))
        // a revolved or screwed profile must keep to one side of the axis
        || (is(&["revolve", "screw"]) && detail.contains("axis crosses the profile"))
        // a screw's turns must stay apart
        || (op == "screw" && detail.contains("coils meet"))
        // a torus needs major > minor > 0
        || (op == "torus" && detail.contains("clear of its axis"))
        // a shell's walls must not meet; a moved face may not pass the face it meets
        || (is(&["shell", "offset_faces", "move_faces"]) && detail.contains("turns inside out"))
}
