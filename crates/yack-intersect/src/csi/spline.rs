// yack: a line against a spline surface, by Bézier patches cut down until each holds one crossing.
// Adapted from springrep@a03e8a1 src/boolean/csi.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: split out of csi.rs (M3 hardening): named limits and coordinates, the patch's slopes as
// named rows instead of indexed matrices; the same arithmetic in the same order.
//! A line against a spline surface, where no one parameter carries the problem: the surface's
//! Bézier patches, read in the line's own coordinates, are dropped where they stand clear of the
//! line, solved where they hold one crossing at most, and halved otherwise.

use super::{
    Hits,
    roots::{UNDERFLOW, sampled_roots},
};
use crate::{
    geom::{Bezier, NurbsSurface},
    math::{Aabb, Point3, Vec2, Vec3},
    numeric, tol,
};

/// The most patches of the tolerance's size [`line_spline`] finds along a line before it takes the
/// line to run in the surface: a touch or a pair of close crossings comes to a few such patches,
/// and a line that stays within the tolerance of the surface comes to as many as its length holds.
const SPLINE_RUN: usize = 64;

/// The most patches [`line_spline`] opens in all, a bound on its work whatever the surface.
const SPLINE_PATCHES: usize = 20_000;

/// A patch whose parameter span is this small in both directions is spent: halving it moves
/// nothing.
const SPENT_SPAN: f64 = 1e-14; // tol: parameter rounding

/// Two crossings this share of the tolerance apart along the line are one, found from two patches.
const SAME_CROSSING: f64 = 1e-3; // tol: a share of tol::LINEAR

/// A patch's slopes are singular where their determinant is below this share of their size.
const SINGULAR_SLOPES: f64 = 1e-9; // tol: a relative determinant

/// Newton's method on a patch stops after this many steps, or a step this small.
const NEWTON_STEPS: usize = 40;
const NEWTON_STILL: f64 = 1e-14; // tol: parameter rounding

/// A patch's parameters this far outside [0, 1] are outside it.
const PATCH_ROUNDING: f64 = 1e-9; // tol: patch parameter rounding

/// The fewest and most samples along each stretch of the line near the surface.
const STRETCH_SAMPLES: (usize, usize) = (2, 32);

/// A line against a spline surface, where no one parameter carries the problem.
///
/// The surface is cut into Bézier patches, each read in the line's own coordinates: its distance
/// from two planes through the line, and its place along the line, each a polynomial once
/// multiplied by the patch's weight. A patch lies among its control points, and so does each half
/// of a patch cut in two. So a patch whose control points all stand to one side of either plane,
/// or all beyond one end of the range, holds no crossing, and is dropped. Of what is left:
///
/// - A patch whose two distances cannot both vanish twice is solved outright. That is so where
///   their slopes, bounded by the steps between its control points, make the map from the
///   patch's parameters to the two distances one to one.
/// - Any other is halved, down to patches the size of the tolerance, which a touch or a pair of
///   crossings a hair apart comes to.
///
/// Every crossing lies in a patch that is solved or runs down to that size, so none is stepped
/// over however close two lie. `None` where the patches of that size run on along the line: a
/// line that lies in the surface, which has no crossings to find and is left to the caller.
pub(super) fn line_spline(
    o: Point3,
    d: Vec3,
    ns: &NurbsSurface,
    range: (f64, f64),
) -> Option<Hits> {
    // Two directions square to the line and to each other.
    let side = if d.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
    let e1 = d.cross(side).unit()?;
    let e2 = d.cross(e1);
    // A control point as its distance from each plane and its place along the line, all times
    // its weight, and the weight.
    let own = |[x, y, z, w]: [f64; 4]| -> [f64; 4] {
        let rel = Vec3::new(x, y, z) - o * w;
        [rel.dot(e1), rel.dot(e2), rel.dot(d), w]
    };
    // Whether every control point stands clear of zero in one of the two distances on one side,
    // or beyond an end of the range along the line: the patch then holds no crossing in the range.
    let clear = |patch: &Bezier| {
        let pts = patch.points();
        let beyond = |f: &dyn Fn(&[f64; 4]) -> f64| pts.iter().all(|q| f(q) > 0.0);
        let clear_of = |c: &dyn Fn(&[f64; 4]) -> f64| {
            beyond(&|q| c(q) - tol::LINEAR * q[3]) || beyond(&|q| -c(q) - tol::LINEAR * q[3])
        };
        clear_of(&|q| q[0])
            || clear_of(&|q| q[1])
            || beyond(&|q| q[2] - (range.1 + tol::LINEAR) * q[3])
            || beyond(&|q| (range.0 - tol::LINEAR) * q[3] - q[2])
    };
    let mut open: Vec<Bezier> = ns
        .beziers()
        .iter()
        .filter(|b| line_passes(o, d, &b.bbox(), range))
        .map(|b| b.mapped(own))
        .filter(|b| !clear(b))
        .collect();
    let mut ts: Vec<f64> = Vec::new();
    // Each patch of the tolerance's size the line passes: its stretch of the line, and where on
    // the surface it lies.
    let mut small: Vec<(f64, f64, Vec2)> = Vec::new();
    let mut opened = 0;
    while let Some(patch) = open.pop() {
        opened += 1;
        if opened > SPLINE_PATCHES {
            return None;
        }
        if let Some(t) = only_crossing(&patch) {
            if t >= range.0 - tol::LINEAR && t <= range.1 + tol::LINEAR {
                ts.push(t);
            }
            continue;
        }
        let bb = patch.bbox();
        let span = patch.hi - patch.lo;
        let spent = span.x.abs().max(span.y.abs()) <= SPENT_SPAN;
        if bb.diagonal() <= tol::LINEAR || spent {
            small.push((bb.min.z, bb.max.z, (patch.lo + patch.hi) * 0.5));
            if small.len() > SPLINE_RUN {
                return None;
            }
            continue;
        }
        open.extend(patch.halves(patch.longer_in_u()).into_iter().filter(|h| !clear(h)));
    }
    small.sort_by(|x, y| x.0.total_cmp(&y.0));
    let mut stretches: Vec<(f64, f64, Vec2)> = Vec::new();
    for (a, b, uv) in small {
        match stretches.last_mut() {
            Some(last) if a <= last.1 + tol::LINEAR => last.1 = last.1.max(b),
            _ => stretches.push((a, b, uv)),
        }
    }
    for (a, b, hint) in stretches {
        // The line's height over the surface near the patch: positive on the side of its normal.
        let mut last = hint;
        let mut height = |t: f64| {
            let q = o + d * t;
            let uv = ns.project_near(q, last);
            last = uv;
            let (at, su, sv) = ns.d1(uv.x, uv.y);
            match su.cross(sv).unit() {
                Some(n) => (q - at).dot(n),
                // At a pole no normal is defined: the distance, which is zero on the surface.
                None => q.dist(at),
            }
        };
        let (a, b) = ((a - tol::LINEAR).max(range.0), (b + tol::LINEAR).min(range.1));
        let (fewest, most) = STRETCH_SAMPLES;
        let steps = (((b - a) / tol::LINEAR).ceil() as usize).clamp(fewest, most);
        ts.extend(sampled_roots(&mut height, &[a, b], steps).0);
    }
    // A crossing on the side two patches share is found from both, a rounding apart.
    ts.sort_by(f64::total_cmp);
    ts.dedup_by(|a, b| (*a - *b).abs() <= SAME_CROSSING * tol::LINEAR);
    Some(Hits { ts, on_surface: false })
}

/// Whether the line `o + t·d` passes within the tolerance of the box for some `t` in the range.
fn line_passes(o: Point3, d: Vec3, bb: &Aabb, range: (f64, f64)) -> bool {
    let (mut lo, mut hi) = range;
    let (min, max) = (bb.min.to_array(), bb.max.to_array());
    for (((min, max), ok), dk) in min.into_iter().zip(max).zip(o.to_array()).zip(d.to_array()) {
        let (mn, mx) = (min - tol::LINEAR, max + tol::LINEAR);
        if dk.abs() < UNDERFLOW {
            if ok < mn || ok > mx {
                return false;
            }
            continue;
        }
        let (a, b) = ((mn - ok) / dk, (mx - ok) / dk);
        lo = lo.max(a.min(b));
        hi = hi.min(a.max(b));
    }
    hi >= lo
}

/// The range of `x·a + y·b` for `a` and `b` each in a range.
fn mixed(x: f64, a: (f64, f64), y: f64, b: (f64, f64)) -> (f64, f64) {
    let part = |k: f64, r: (f64, f64)| (k * r.0).min(k * r.1)..=(k * r.0).max(k * r.1);
    let (p, q) = (part(x, a), part(y, b));
    (p.start() + q.start(), p.end() + q.end())
}

/// Where the line crosses a patch read in the line's coordinates (its distances from two planes
/// through the line, its place along the line, its weight), when the patch can hold one crossing
/// at most and holds one. `None` where it may hold more, or holds none.
///
/// The two distances, as functions of the patch's parameters, cannot both vanish twice where that
/// map is one to one, and it is where its matrix of slopes, turned by a fixed matrix, keeps a
/// positive definite symmetric part: then two points of the patch always map apart. The slopes
/// are bounded by the steps between control points, so the bound holds all over the patch.
fn only_crossing(patch: &Bezier) -> Option<f64> {
    // The ranges of each distance's slope along each parameter: `first[c]` is the first
    // distance's along parameter `c`.
    let (first, second) = (patch.slopes(0), patch.slopes(1));
    let mid = |r: (f64, f64)| 0.5 * (r.0 + r.1);
    // The slopes at their middle, and the turn that makes them the identity there.
    let [[m00, m01], [m10, m11]] = [first.map(mid), second.map(mid)];
    let det = m00 * m11 - m01 * m10;
    let size = m00.abs().max(m01.abs()) * m10.abs().max(m11.abs());
    if det.abs() <= SINGULAR_SLOPES * size || size == 0.0 {
        return None;
    }
    let turn = [[m11 / det, -m01 / det], [-m10 / det, m00 / det]];
    let turned = |[t0, t1]: [f64; 2], col: usize| -> Option<(f64, f64)> {
        Some(mixed(t0, *first.get(col)?, t1, *second.get(col)?))
    };
    let [top, bottom] = turn;
    let (a, b, c, e) = (turned(top, 0)?, turned(top, 1)?, turned(bottom, 0)?, turned(bottom, 1)?);
    let off = b.0.abs().max(b.1.abs()) + c.0.abs().max(c.1.abs());
    if !(a.0 > 0.0 && e.0 > 0.0 && off * off < 4.0 * a.0 * e.0) {
        return None;
    }
    // One crossing at most: Newton's method from the middle finds it, or it lies outside.
    let (mut s, mut t) = (0.5, 0.5);
    for _ in 0..NEWTON_STEPS {
        let [q, qs, qt] = patch.at(s, t);
        let (ds, dt) = numeric::solve2(qs[0], qt[0], qs[1], qt[1], -q[0], -q[1])?;
        (s, t) = (s + ds, t + dt);
        if !(-0.5..=1.5).contains(&s) || !(-0.5..=1.5).contains(&t) {
            return None;
        }
        if ds.abs().max(dt.abs()) <= NEWTON_STILL {
            break;
        }
    }
    let inside = |x: f64| (-PATCH_ROUNDING..=1.0 + PATCH_ROUNDING).contains(&x);
    if !inside(s) || !inside(t) {
        return None;
    }
    let [q, ..] = patch.at(s.clamp(0.0, 1.0), t.clamp(0.0, 1.0));
    // On the line within rounding, or the search settled short of it.
    let off = q[0].hypot(q[1]) / q[3];
    (off <= SAME_CROSSING * tol::LINEAR * (1.0 + (q[2] / q[3]).abs())).then_some(q[2] / q[3])
}
