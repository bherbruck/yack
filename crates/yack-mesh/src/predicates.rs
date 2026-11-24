// yack: Plane predicates for the triangulation. The orientation test is exact, so point location, the
// Adapted from springrep@a03e8a1 src/mesh/predicates.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/mesh/predicates.rs to yack-mesh (M3 W6); paths and visibility only.
//! Plane predicates for the triangulation. The orientation test is exact, so point location, the
//! convexity of a flip and the side of a constraint never contradict one another; the in-circle
//! test only decides which of two valid triangulations to keep, so a rounded answer is enough.
//!
//! The error bound and the expansion sum are those of J. R. Shewchuk, "Adaptive precision
//! floating-point arithmetic and fast robust geometric predicates", Discrete & Computational
//! Geometry 18, 1997, pp. 305–363.

use crate::math::Vec2;

/// Twice the signed area of (a, b, c): positive when counter-clockwise, exactly zero only when
/// the three points are collinear.
pub(super) fn orient(a: Vec2, b: Vec2, c: Vec2) -> f64 {
    let l = (a.x - c.x) * (b.y - c.y);
    let r = (a.y - c.y) * (b.x - c.x);
    let det = l - r;
    // Shewchuk's first error bound for this form, rounded up.
    let bound = 3.4e-16 * (l.abs() + r.abs());
    if det.abs() > bound {
        return det;
    }
    exact_orient(a, b, c)
}

/// The sign of the orientation determinant from its six products, each split exactly into two
/// doubles and summed as a non-overlapping expansion; the largest component carries the sign.
fn exact_orient(a: Vec2, b: Vec2, c: Vec2) -> f64 {
    let terms = [
        (a.x, b.y),
        (-a.x, c.y),
        (-c.x, b.y),
        (-a.y, b.x),
        (a.y, c.x),
        (c.y, b.x),
    ];
    let mut e: Vec<f64> = Vec::with_capacity(12);
    for (p, q) in terms {
        let hi = p * q;
        let lo = p.mul_add(q, -hi);
        grow(&mut e, lo);
        grow(&mut e, hi);
    }
    e.last().copied().unwrap_or(0.0)
}

/// Shewchuk's Grow-Expansion with zero elimination: adds `b` to the expansion `e`, which stays
/// non-overlapping and ordered by increasing magnitude.
fn grow(e: &mut Vec<f64>, b: f64) {
    let mut q = b;
    let mut out = Vec::with_capacity(e.len() + 1);
    for &x in e.iter() {
        let s = q + x;
        let bv = s - q;
        let err = (q - (s - bv)) + (x - bv);
        if err != 0.0 {
            out.push(err);
        }
        q = s;
    }
    if q != 0.0 {
        out.push(q);
    }
    *e = out;
}

/// Whether `d` lies clearly inside the circle through the counter-clockwise triangle (a, b, c).
pub(super) fn in_circle(a: Vec2, b: Vec2, c: Vec2, d: Vec2) -> bool {
    let (ax, ay) = (a.x - d.x, a.y - d.y);
    let (bx, by) = (b.x - d.x, b.y - d.y);
    let (cx, cy) = (c.x - d.x, c.y - d.y);
    let (a2, b2, c2) = (ax * ax + ay * ay, bx * bx + by * by, cx * cx + cy * cy);
    let det = a2 * (bx * cy - cx * by) - b2 * (ax * cy - cx * ay) + c2 * (ax * by - bx * ay);
    let scale = a2 * (bx * cy).abs().max((cx * by).abs())
        + b2 * (ax * cy).abs().max((cx * ay).abs())
        + c2 * (ax * by).abs().max((bx * ay).abs());
    det > 1e-12 * scale
}

/// The centre of the circle through (a, b, c), or `None` when they are collinear.
pub(super) fn circumcenter(a: Vec2, b: Vec2, c: Vec2) -> Option<Vec2> {
    let (b, c) = (b - a, c - a);
    let d = 2.0 * b.cross(c);
    if d == 0.0 {
        return None;
    }
    let (b2, c2) = (b.dot(b), c.dot(c));
    let center = Vec2::new(c.y * b2 - b.y * c2, b.x * c2 - c.x * b2) / d;
    let out = a + center;
    (out.x.is_finite() && out.y.is_finite()).then_some(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn orientation_is_exact_on_nearly_collinear_points() {
        let a = Vec2::new(0.5, 0.5);
        let b = Vec2::new(12.0, 12.0);
        let c = Vec2::new(24.0, 24.0);
        assert_eq!(orient(a, b, c), 0.0);
        let tiny = Vec2::new(0.5 + f64::EPSILON, 0.5);
        assert!(orient(tiny, b, c) < 0.0);
        let up = Vec2::new(0.5, 0.5 + f64::EPSILON);
        assert!(orient(up, b, c) > 0.0);
        assert!(
            orient(
                Vec2::new(0.0, 0.0),
                Vec2::new(1.0, 0.0),
                Vec2::new(0.0, 1.0)
            ) > 0.0
        );
    }

    #[test]
    fn circumcenter_of_a_right_triangle_is_its_hypotenuse_midpoint() {
        let c = circumcenter(
            Vec2::new(0.0, 0.0),
            Vec2::new(2.0, 0.0),
            Vec2::new(0.0, 2.0),
        );
        assert_eq!(c, Some(Vec2::new(1.0, 1.0)));
        assert!(in_circle(
            Vec2::new(0.0, 0.0),
            Vec2::new(2.0, 0.0),
            Vec2::new(0.0, 2.0),
            Vec2::new(1.0, 1.0)
        ));
    }
}
