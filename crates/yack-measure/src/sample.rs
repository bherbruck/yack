// yack: An edge's length, points spread over a face, how far a face or a solid reaches along a
// Adapted from springrep@a03e8a1 src/measure/sample.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/measure/sample.rs to yack-measure (M3 W5); paths and visibility only.
//! An edge's length, points spread over a face, how far a face or a solid reaches along a
//! direction, and one solid's box and area.

use super::{face_area, face_box, reached_patches, stationary, uv_polygons};
use crate::{
    topo::face_domain,
    geom::Surface,
    math::{Aabb, Point3, Vec2, Vec3},
    numeric::integrate,
    topo::{Brep, EdgeId, FaceId, SolidId},
};

/// The length of edge `e`, from its exact curve.
pub fn edge_length(b: &Brep, e: EdgeId) -> f64 {
    let edge = b.edge(e);
    let Some(c) = &edge.curve else { return 0.0 };
    let (t0, t1) = edge.range;
    let mut cuts = vec![t0];
    cuts.extend(c.breakpoints(t0.min(t1), t0.max(t1)));
    cuts.push(t1);
    cuts.sort_by(f64::total_cmp);
    let mut speed = |t: f64| c.d1(t).1.norm();
    let pieces = cuts.windows(2);
    pieces
        .filter_map(|w| match *w {
            [a, z] => Some(integrate(&mut speed, a, z, 1e-10)),
            _ => None,
        })
        .sum()
}

/// The box of face `f`'s parameters: the least and greatest `(u, v)` its outline reaches.
pub fn face_uv_bounds(b: &Brep, f: FaceId) -> (Vec2, Vec2) {
    let domain = face_domain(b, f);
    let (mut lo, mut hi) = (
        Vec2::new(f64::INFINITY, f64::INFINITY),
        Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY),
    );
    for uv in domain.samples().iter().flatten().flatten() {
        lo = Vec2::new(lo.x.min(uv.x), lo.y.min(uv.y));
        hi = Vec2::new(hi.x.max(uv.x), hi.y.max(uv.y));
    }
    (lo, hi)
}

/// How far face `f` reaches along `dir`: the least and greatest of `p · dir` over its points,
/// `dir` made a unit, from the exact geometry. Its edges' extremes, and where a curved face
/// bulges past them, the point where it turns.
pub fn face_extent(b: &Brep, f: FaceId, dir: Vec3) -> (f64, f64) {
    let Some(d) = dir.unit() else {
        return (0.0, 0.0);
    };
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    let mut reach = |p: Point3| {
        let s = (p - Point3::new(0.0, 0.0, 0.0)).dot(d);
        (lo, hi) = (lo.min(s), hi.max(s));
    };
    let face = b.face(f);
    for &lp in &face.loops {
        for &co in &b.lp(lp).coedges {
            let edge = b.edge(b.coedge(co).edge);
            reach(b.vertex(edge.start).point);
            reach(b.vertex(edge.end).point);
            if let Some(c) = &edge.curve {
                for t in c.extremes(d, edge.range.0, edge.range.1) {
                    reach(c.eval(t));
                }
            }
        }
    }
    let s = &face.surface;
    let bulges = matches!(
        s,
        Surface::Sphere(_)
            | Surface::Torus(_)
            | Surface::Revolution(_)
            | Surface::Ruled(_)
            | Surface::Nurbs(_)
    );
    if bulges {
        let polys = uv_polygons(b, f);
        // A B-spline lies inside its patches' boxes: only a patch that reaches past what the
        // edges reach can widen the extent, and only there is a turning point looked for.
        let starts = match s {
            Surface::Nurbs(n) => {
                let mut starts = Vec::new();
                for q in reached_patches(n, &polys) {
                    let (bl, bh) = (q.bbox.min, q.bbox.max);
                    let along = |k: usize| {
                        let (hi, lo) = (bh.to_array(), bl.to_array());
                    let pick = |i: usize| {
                        let corner = if k >> i & 1 == 1 { hi } else { lo };
                        corner.get(i).copied().unwrap_or_default()
                    };
                        pick(0) * d.x + pick(1) * d.y + pick(2) * d.z
                    };
                    let (ql, qh) = (0..8)
                        .map(along)
                        .fold((f64::INFINITY, f64::NEG_INFINITY), |a, x| {
                            (a.0.min(x), a.1.max(x))
                        });
                    if ql >= lo && qh <= hi {
                        continue;
                    }
                    for i in 0..3 {
                        for j in 0..3 {
                            let at =
                                |a: f64, z: f64, k: usize| a + (z - a) * (k as f64 + 0.5) / 3.0;
                            starts.push(Vec2::new(at(q.lo.x, q.hi.x, i), at(q.lo.y, q.hi.y, j)));
                        }
                    }
                }
                Some(starts)
            }
            _ => None,
        };
        if starts.as_ref().is_none_or(|s| !s.is_empty()) {
            let turning = stationary(s, &polys, d, starts);
            let mut reach = |p: Point3| {
                let x = (p - Point3::new(0.0, 0.0, 0.0)).dot(d);
                (lo, hi) = (lo.min(x), hi.max(x));
            };
            for uv in turning {
                reach(s.eval(uv.x, uv.y));
            }
        }
    }
    (lo, hi)
}

/// How far the arena reaches along `dir`: the least and greatest of `p · dir` over its faces.
pub fn extent(b: &Brep, dir: Vec3) -> (f64, f64) {
    let each = b.face_ids().map(|f| face_extent(b, f, dir));
    each.fold((f64::INFINITY, f64::NEG_INFINITY), |a, e| {
        (a.0.min(e.0), a.1.max(e.1))
    })
}

/// A box holding solid `s` of the arena: the smallest, but where it has B-spline faces, which may
/// widen it a little.
pub fn solid_bbox(b: &Brep, s: SolidId) -> Aabb {
    let mut bb = Aabb::EMPTY;
    for f in b.solid_faces(s) {
        bb.union(&face_box(b, f));
    }
    bb
}

/// The area of solid `s`'s faces.
pub fn solid_area(b: &Brep, s: SolidId) -> f64 {
    b.solid_faces(s).into_iter().map(|f| face_area(b, f)).sum()
}

/// Points spread over face `f`, each with the solid's outward normal there: the points of an
/// `n × n` grid across the face's parameters that fall inside it. A face narrower than the grid's
/// spacing can come back with none.
pub fn face_samples(b: &Brep, f: FaceId, n: usize) -> Vec<(Point3, Vec3)> {
    let domain = face_domain(b, f);
    let face = b.face(f);
    let (lo, hi) = face_uv_bounds(b, f);
    let mut out = Vec::new();
    for i in 0..n {
        for j in 0..n {
            let at = |k: usize, a: f64, b: f64| a + (k as f64 + 0.5) / n as f64 * (b - a);
            let uv = Vec2::new(at(i, lo.x, hi.x), at(j, lo.y, hi.y));
            if domain.winding(uv) == 0 {
                continue;
            }
            let normal = face.surface.normal(uv.x, uv.y);
            let outward = if face.reversed { -normal } else { normal };
            out.push((face.surface.eval(uv.x, uv.y), outward));
        }
    }
    out
}
