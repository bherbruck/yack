// yack: measured guards on a solid's shape, and a face's grid (moved from springrep's facade/api.rs, M3 W10).
//! Guards the facade runs before an operation, from a solid's measures: shapes springrep would build
//! wrong, which its rung must decline (M1.10), and a face's grid of points.

use crate::math::{Point3, Vec3};
use crate::topo::{Brep, FaceId};

/// An even `nu` × `nv` grid of points over face `f`'s parameter box (rows along u). The box
/// spans the face's outline, so on a trimmed face some points lie outside the face itself.
pub fn face_grid(b: &Brep, f: FaceId, nu: usize, nv: usize) -> Vec<Vec<Point3>> {
    let (lo, hi) = crate::face_uv_bounds(b, f);
    let s = &b.face(f).surface;
    let at = |i: usize, n: usize, a: f64, z: f64| if n <= 1 { a } else { a + (z - a) * i as f64 / (n - 1) as f64 };
    (0..nu).map(|i| (0..nv).map(|j| s.eval(at(i, nu, lo.x, hi.x), at(j, nv, lo.y, hi.y))).collect()).collect()
}

/// Whether `a` and `b` have the same face count, volume and area (to the linear tolerance,
/// relative): an edit that "succeeded" without changing the solid.
pub fn same_shape(a: &Brep, b: &Brep) -> bool {
    let close = |x: f64, y: f64| (x - y).abs() <= crate::tol::LINEAR * x.abs().max(y.abs()).max(1.0);
    a.face_count() == b.face_count()
        && close(crate::volume(a), crate::volume(b))
        && close(crate::area(a), crate::area(b))
}

/// Whether an inward shell of `thickness` makes two flat walls meet or cross: two planar faces (not
/// `open`) facing each other across material, no farther apart than twice the thickness, whose
/// boxes overlap across the gap.
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
            // Walls that only meet leave no cavity between them either: as bad as crossing.
            if d <= 0.0 || d > gap + crate::tol::LINEAR {
                return false;
            }
            let (bf, bg) = (crate::face_bbox(b, f), crate::face_bbox(b, g));
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
    let (lo, hi) = crate::extent(b, n);
    let at = origin.dot(n);
    lo < at - crate::tol::LINEAR && hi > at + crate::tol::LINEAR
}

/// Whether the bounding boxes of `a` and `b` overlap with positive volume (by more than the
/// linear tolerance on every axis).
pub fn boxes_overlap(a: &Brep, b: &Brep) -> bool {
    let (p, q) = (crate::bbox(a), crate::bbox(b));
    let t = crate::tol::LINEAR;
    p.min.x + t < q.max.x && q.min.x + t < p.max.x && p.min.y + t < q.max.y && q.min.y + t < p.max.y && p.min.z + t < q.max.z && q.min.z + t < p.max.z
}
