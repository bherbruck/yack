// yack: bounding boxes of faces and curves, and the grid cell of a point.
// Adapted from springrep@a03e8a1 src/boolean/mod.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/boolean/mod.rs to yack-boolean (M3 W8); split out of lib.rs (M3
// hardening B0), paths and visibility only; its one small literal named (M3 B1).
//! Bounding boxes of faces and curves, and the grid cell of a point.

use super::*;

/// What a face's box is grown by, as a share of its size plus a millimetre: room for the boundary's
/// sampled images to fall a hair short of the face's extremes.
const FACE_BOX_SLACK: f64 = 1e-6; // tol: a share of the box's size, not a feature size

pub(super) fn cell(p: Point3) -> [i64; 3] {
    [
        (p.x / CELL).floor() as i64,
        (p.y / CELL).floor() as i64,
        (p.z / CELL).floor() as i64,
    ]
}

pub(super) fn overlap(a: &Aabb, b: &Aabb) -> bool {
    !a.is_empty() && !b.is_empty() && (0..3).all(|k| a.min[k] <= b.max[k] && b.min[k] <= a.max[k])
}

pub(super) fn padded(b: &Aabb, pad: f64) -> Aabb {
    Aabb {
        min: b.min - Vec3::new(pad, pad, pad),
        max: b.max + Vec3::new(pad, pad, pad),
    }
}

pub(super) fn curve_bbox(c: &Curve, range: (f64, f64)) -> Aabb {
    c.bbox(range.0, range.1)
}

/// Adds to `bb` every point of a sphere or torus about `frame` inside the face's parameter window
/// where a world coordinate is stationary: a face's extremes lie on its boundary or at those. The
/// window is widened by a tenth, since it is the box of the boundary's sampled images.
pub(super) fn stationary_points(s: &Surface, frame: &Frame, dom: &Domain, bb: &mut Aabb) {
    let pad = (dom.hi - dom.lo) * 0.1;
    let (lo, hi) = (dom.lo - pad, dom.hi + pad);
    let images = |x: f64, lo: f64, hi: f64, period: Option<f64>| -> Vec<f64> {
        let Some(p) = period else {
            return if (lo..=hi).contains(&x) {
                vec![x]
            } else {
                Vec::new()
            };
        };
        let mut c = x + ((lo - x) / p).ceil() * p;
        let mut out = Vec::new();
        while c <= hi {
            out.push(c);
            c += p;
        }
        out
    };
    for d in [Vec3::X, Vec3::Y, Vec3::Z] {
        let (dx, dy, dz) = (d.dot(frame.x), d.dot(frame.y), d.dot(frame.z));
        // Along the axis every angle is stationary, and any one stands for the rest.
        let us = if dx.hypot(dy) <= f64::EPSILON {
            vec![0.5 * (lo.x + hi.x)]
        } else {
            let a = dy.atan2(dx);
            vec![a, a + PI]
        };
        for u in us
            .into_iter()
            .flat_map(|u| images(u, lo.x, hi.x, s.u_period()))
        {
            let b = dz.atan2(dx * u.cos() + dy * u.sin());
            for v in [b - PI, b, b + PI] {
                for v in images(v, lo.y, hi.y, s.v_period()) {
                    bb.add(s.eval(u, v));
                }
            }
        }
    }
}

/// A box that holds the face: its boundary's, with whatever the surface can reach past it. The
/// one box of a face for every cull against it, so a ray that leaves through a face's bulge is
/// not dropped by a box that stops short of it.
///
/// A plane, and any surface drawn by straight lines (a cylinder, a cone, an extrusion along its
/// direction, a ruled surface), reaches no further than its boundary: its extremes lie along its
/// lines, which run to the boundary. A sphere and a torus have their stationary points in closed
/// form. A surface of revolution lies on the rings of its profile, inside the cylinder that holds
/// them. A spline surface lies in its patches' boxes. The parameter window is widened by a tenth,
/// since it is the box of the boundary's sampled images.
pub(crate) fn face_bbox(dom: &Domain) -> Aabb {
    let mut bb = Aabb::EMPTY;
    for tr in dom.trims() {
        bb.union(&tr.bbox());
    }
    let s = &dom.surface;
    let pad = (dom.hi - dom.lo) * 0.1;
    let (lo, hi) = (dom.lo - pad, dom.hi + pad);
    match s {
        Surface::Sphere(sp) => stationary_points(s, &sp.frame, dom, &mut bb),
        Surface::Torus(t) => stationary_points(s, &t.frame, dom, &mut bb),
        Surface::Revolution(r) => {
            // The profile's piece lies in its own box, so its rings lie between the box's least
            // and greatest heights along the axis, no further from the axis than its furthest
            // corner: a distance from a line is greatest over a box at a corner.
            let profile = r.curve.bbox(lo.y, hi.y);
            let (mut low, mut high, mut reach) = (f64::INFINITY, f64::NEG_INFINITY, 0.0f64);
            for k in 0..8 {
                let pick = |bit: usize, a: f64, b: f64| if k >> bit & 1 == 0 { a } else { b };
                let corner = Vec3::new(
                    pick(0, profile.min.x, profile.max.x),
                    pick(1, profile.min.y, profile.max.y),
                    pick(2, profile.min.z, profile.max.z),
                ) - r.origin;
                let z = corner.dot(r.axis);
                (low, high) = (low.min(z), high.max(z));
                reach = reach.max((corner - r.axis * z).norm());
            }
            // A disc of that reach square to the axis at each end.
            let spread = Vec3::new(
                reach * (1.0 - r.axis.x * r.axis.x).max(0.0).sqrt(),
                reach * (1.0 - r.axis.y * r.axis.y).max(0.0).sqrt(),
                reach * (1.0 - r.axis.z * r.axis.z).max(0.0).sqrt(),
            );
            for z in [low, high] {
                let centre = r.origin + r.axis * z;
                bb.add(centre - spread);
                bb.add(centre + spread);
            }
        }
        Surface::Nurbs(ns) => {
            // A periodic parameter's window may lie a period off the patches' own.
            let periodic = ns.u_period().is_some() || ns.v_period().is_some();
            for q in ns.patches() {
                let apart = q.hi.x < lo.x || q.lo.x > hi.x || q.hi.y < lo.y || q.lo.y > hi.y;
                if periodic || !apart {
                    bb.union(&q.bbox);
                }
            }
        }
        _ => {}
    }
    padded(&bb, FACE_BOX_SLACK * (1.0 + bb.diagonal()))
}

/// Where a piece of curve through `[start, middle, end]` lies in a face: as its middle does,
/// except that a piece not running along one trim is inside where the exact side puts its middle.
/// Shorter than twice the tolerance, a piece leaving the boundary, or cutting across a corner, has
/// its middle within the tolerance of the boundary, though only one lying along a trim is on it.
pub(super) fn locate_piece(dom: &Domain, pts: [Point3; 3]) -> Where {
    match dom.locate(pts[1]) {
        Where::Boundary if !dom.along_one_trim(&pts) => dom
            .exact_side(pts[1])
            .map_or(Where::Boundary, Where::Inside),
        w => w,
    }
}
