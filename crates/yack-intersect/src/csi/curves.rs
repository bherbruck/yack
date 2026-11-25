// yack: where two curves meet, where curves graze, and points on three surfaces.
// Adapted from springrep@a03e8a1 src/boolean/csi.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: split out of csi.rs (M3 hardening): the line and coplanar-circle cases as their own
// functions, a span index that falls back to the plain projection instead of panicking when empty,
// samples read by window and checked access instead of indexing; the same arithmetic, and the same
// curve evaluations, in the same order.
//! Where two curves meet ([`curve_curve`]), where two curves running a hair apart cross
//! ([`grazing`]), the meeting of a curve with an edge near a known point ([`curve_curve_near`]),
//! and the point on three surfaces near a guess ([`on_three`]).

use super::{
    conic::{conic_plane, line_conic},
    curve_distance, curve_surface, in_range,
    roots::{evenly, evenly_through, minimise, spans, with_ends},
    signed_distance, within,
};
use crate::{
    geom::{Circle, Curve, Line, Plane, Surface},
    hulls,
    math::{Aabb, Frame, Point3, Vec3},
    numeric, tol,
};

/// Two vertices closer than this, mm, along curves running nearly together are one meeting.
pub const GRAZE_GAP: f64 = 1e-5; // tol: a meeting of grazing curves

/// The samples along `c1` [`grazing`] looks for a least gap among.
const GRAZE_SAMPLES: usize = 64;

/// The golden-section steps [`grazing`] settles each least gap with.
const GRAZE_STEPS: usize = 100;

/// A grazing least gap whose neighbours both lie within this share of the tolerance is curves
/// that coincide, apart by rounding alone.
const COINCIDENT: f64 = 1e-3; // tol: a share of tol::LINEAR

/// Gauss–Newton and Newton steps stop after this many, or when a step is this share of
/// `1 + |parameter|`.
const NEAR_STEPS: usize = 40;
const ONTO_STEPS: usize = 30;
const STILL: f64 = 1e-15; // tol: relative parameter rounding

/// A point is on a surface for [`on_three`] and [`onto_surface`] within this share of the
/// tolerance.
const SETTLED: f64 = 1e-3; // tol: a share of tol::LINEAR

/// The samples along each span between `c1`'s breakpoints that the curve–curve search takes.
const SPAN_SAMPLES: usize = 16;

/// A curve's range is split into at least this many spans for its index.
const INDEX_SPANS: usize = 8;

/// The golden ratio's conjugate, the shrink of a golden-section bracket per step.
fn golden() -> f64 {
    0.5 * (5f64.sqrt() - 1.0)
}

/// Where `c1` crosses `c2` while the two run within [`GRAZE_GAP`] of each other: a local least of
/// the gap over 64 samples, settled by golden-section search, where the offset from one to the
/// other turns round. Two splines a hair apart cross at angles of 1e-7 rad, where a general step
/// loses them; a touch without a crossing is left alone.
pub fn grazing(c1: &Curve, r1: (f64, f64), c2: &Curve, r2: (f64, f64)) -> Vec<Point3> {
    let gap = |t: f64| curve_distance(c2, r2, c1.eval(t)).0;
    let ts: Vec<f64> = evenly_through(r1.0, r1.1, GRAZE_SAMPLES).collect();
    // A sample on no piece of `c1` near `c2` is farther than the graze from it, and so never
    // starts a search, nor lets a neighbour start one.
    let reach = hulls::near(c1, r1, c2, r2, GRAZE_GAP + 2.0 * tol::LINEAR);
    if reach.as_ref().is_some_and(Vec::is_empty) {
        return Vec::new();
    }
    let ds: Vec<f64> = ts
        .iter()
        .map(|&t| match &reach {
            Some(r) if !r.iter().any(|&(lo, hi)| lo <= t && t <= hi) => f64::INFINITY,
            _ => gap(t),
        })
        .collect();
    let offset = |s: f64| {
        let p = c1.eval(s);
        c2.eval(curve_distance(c2, r2, p).1) - p
    };
    let mut out = Vec::new();
    for (params, gaps) in ts.windows(3).zip(ds.windows(3)) {
        let (&[before, _, after], &[d_before, d_here, d_after]) = (params, gaps) else {
            continue;
        };
        // Both sides apart by more than rounding, so the offset's turn is a crossing: curves that
        // coincide (a symmetric solid turned onto itself) have offsets that are rounding alone.
        let (lo, hi) = (d_before.min(d_after), d_before.max(d_after));
        if d_here > d_before || d_here > d_after || hi > GRAZE_GAP || lo <= COINCIDENT * tol::LINEAR
        {
            continue;
        }
        let (mut a, mut b) = (before, after);
        let g = golden();
        for _ in 0..GRAZE_STEPS {
            let (x1, x2) = (b - g * (b - a), a + g * (b - a));
            if gap(x1) <= gap(x2) {
                b = x2;
            } else {
                a = x1;
            }
        }
        let t = 0.5 * (a + b);
        let crosses = offset(before).dot(offset(after)) < 0.0;
        if gap(t) <= 0.5 * tol::LINEAR && crosses {
            out.push(c1.eval(t));
        }
    }
    out
}

/// The points where two curves meet, transversally or touching; nothing where they overlap, since
/// an overlap's ends are vertices of one curve or the other.
pub fn curve_curve(c1: &Curve, r1: (f64, f64), c2: &Curve, r2: (f64, f64)) -> Vec<Point3> {
    let on2 = |p: Point3| within(c2, r2, p, tol::LINEAR);
    let on1 = |p: Point3| within(c1, r1, p, tol::LINEAR);
    let (plane1, plane2) = (conic_plane(c1), conic_plane(c2));
    match (c1, c2, plane1, plane2) {
        (Curve::Line(a), Curve::Line(b), _, _) => return line_line((a, r1, c1), (b, r2, c2)),
        (Curve::Line(_), _, _, Some(frame)) => return line_conic(c1, r1, c2, r2, frame),
        (_, Curve::Line(_), Some(frame), _) => return line_conic(c2, r2, c1, r1, frame),
        (Curve::Circle(a), Curve::Circle(b), _, _)
            if a.frame.z.cross(b.frame.z).norm() <= tol::ANGULAR
                && (b.frame.origin - a.frame.origin).dot(a.frame.z).abs() <= tol::LINEAR =>
        {
            return coplanar_circles(a, b).into_iter().filter(|&p| on1(p) && on2(p)).collect();
        }
        _ => {}
    }
    // A conic's plane cuts the other curve at its only candidates for a meeting.
    if let Some(pts) = plane_cut(plane2, c1, r1, on2) {
        return pts;
    }
    if let Some(pts) = plane_cut(plane1, c2, r2, on1) {
        return pts;
    }
    sampled_curve_curve(c1, r1, c2, r2, true)
}

/// The points where `c` over `r` crosses the plane `frame`, kept where `on_other` holds; `None`
/// without a plane, or where `c` lies in it.
fn plane_cut(
    frame: Option<Frame>,
    c: &Curve,
    r: (f64, f64),
    on_other: impl Fn(Point3) -> bool,
) -> Option<Vec<Point3>> {
    let plane = Surface::Plane(Plane { frame: frame? });
    let h = curve_surface(c, r, &plane);
    (!h.on_surface).then(|| h.ts.iter().map(|&t| c.eval(t)).filter(|&p| on_other(p)).collect())
}

/// Where two lines, each with its range and as a curve, meet: their nearest points, where they lie
/// within the tolerance of each other in both ranges.
fn line_line(
    (a, r1, c1): (&Line, (f64, f64), &Curve),
    (b, r2, c2): (&Line, (f64, f64), &Curve),
) -> Vec<Point3> {
    let w = a.origin - b.origin;
    let c = a.dir.dot(b.dir);
    let den = 1.0 - c * c;
    if den.sqrt() <= tol::ANGULAR {
        return Vec::new();
    }
    let (d, e) = (a.dir.dot(w), b.dir.dot(w));
    let s = (c * e - d) / den;
    let t = (e - c * d) / den;
    let (p, q) = (a.origin + a.dir * s, b.origin + b.dir * t);
    if p.dist(q) <= tol::LINEAR && in_range(c1, s, r1).is_some() && in_range(c2, t, r2).is_some() {
        vec![p.lerp(q, 0.5)]
    } else {
        Vec::new()
    }
}

/// Where two circles in one plane meet, as whole circles.
fn coplanar_circles(a: &Circle, b: &Circle) -> Vec<Point3> {
    let l = a.frame.to_local(b.frame.origin);
    let d = l.x.hypot(l.y);
    let (ra, rb) = (a.radius, b.radius);
    if d <= tol::LINEAR {
        return Vec::new();
    }
    if d > ra + rb + tol::LINEAR || d < (ra - rb).abs() - tol::LINEAR {
        return Vec::new();
    }
    let ex = Vec3::new(l.x / d, l.y / d, 0.0);
    let ey = Vec3::new(-ex.y, ex.x, 0.0);
    let x = (d * d + ra * ra - rb * rb) / (2.0 * d);
    let touching =
        (d - (ra + rb)).abs() <= tol::LINEAR || (d - (ra - rb).abs()).abs() <= tol::LINEAR;
    let pts: Vec<Vec3> = if touching {
        vec![ex * x.clamp(-ra, ra)]
    } else {
        let y = (ra * ra - x * x).max(0.0).sqrt();
        vec![ex * x + ey * y, ex * x - ey * y]
    };
    pts.into_iter().map(|q| a.frame.to_world(q)).collect()
}

/// Where curve `c` crosses curve `e` near `e`'s point at `t` near `p`, by Gauss–Newton steps on
/// both parameters from there: the point between them, `e`'s parameter, and the sine of the angle
/// they cross at, where they pass within the tolerance of each other; the point is moved along `c`
/// onto `through` where `c` crosses it there. `None` where the curves run together.
pub fn curve_curve_near(
    c: &Curve,
    rc: (f64, f64),
    e: &Curve,
    re: (f64, f64),
    p: Point3,
    t: f64,
    through: Option<&Surface>,
) -> Option<(Point3, f64, f64)> {
    let (mut s, mut t) = (c.project(p, rc.0, rc.1), t);
    for _ in 0..NEAR_STEPS {
        let ((a, da), (b, db)) = (c.d1(s), e.d1(t));
        let r = a - b;
        let (aa, ab, bb) = (da.dot(da), -da.dot(db), db.dot(db));
        let (ds, dt) = numeric::solve2(aa, ab, ab, bb, -da.dot(r), db.dot(r))?;
        let (ns, nt) = ((s + ds).clamp(rc.0, rc.1), (t + dt).clamp(re.0, re.1));
        let still =
            (ns - s).abs() <= STILL * (1.0 + s.abs()) && (nt - t).abs() <= STILL * (1.0 + t.abs());
        (s, t) = (ns, nt);
        if still {
            break;
        }
    }
    let (a, b) = (c.eval(s), e.eval(t));
    // Nearly parallel curves meet along a stretch, not at a point.
    let (da, db) = (c.d1(s).1.unit_or_zero(), e.d1(t).1.unit_or_zero());
    let sine = da.cross(db).norm();
    if a.dist(b) > tol::LINEAR || sine <= tol::TANGENT {
        return None;
    }
    // Moved along `c` onto `through` as well, the point lies on every surface of the meeting, so
    // within each fitted curve's own error of every curve through it.
    let on = through.and_then(|surf| {
        let s = onto_surface(c, rc, s, surf)?;
        let q = c.eval(s);
        let (d, tq) = curve_distance(e, re, q);
        (d <= tol::LINEAR).then_some((q, tq))
    });
    Some(match on {
        Some((q, tq)) => (q, tq, sine),
        None => (a.lerp(b, 0.5), t, sine),
    })
}

/// The point near `p` on all three surfaces, by Newton steps on their tangent planes; `None` where
/// two of them run together there, so the point is not fixed by them.
pub fn on_three(surfs: [&Surface; 3], p: Point3) -> Option<Point3> {
    let planes = |p: Point3| {
        surfs.map(|s| {
            let uv = s.project(p);
            let n = s.normal(uv.x, uv.y);
            (n, (p - s.eval(uv.x, uv.y)).dot(n))
        })
    };
    let mut p = p;
    for _ in 0..ONTO_STEPS {
        let [(n0, d0), (n1, d1), (n2, d2)] = planes(p);
        let det = n0.dot(n1.cross(n2));
        if det.abs() <= tol::TANGENT {
            return None;
        }
        let step = (n1.cross(n2) * -d0 + n2.cross(n0) * -d1 + n0.cross(n1) * -d2) * (1.0 / det);
        p += step;
        if step.norm() <= STILL * (1.0 + p.norm()) {
            break;
        }
    }
    planes(p).iter().all(|&(_, d)| d.abs() <= SETTLED * tol::LINEAR).then_some(p)
}

/// Where curve `c` crosses surface `surf` near parameter `s0`, by Newton steps along it; `None`
/// where it does not cross there steeply enough to converge.
fn onto_surface(c: &Curve, rc: (f64, f64), s0: f64, surf: &Surface) -> Option<f64> {
    let mut s = s0;
    for _ in 0..ONTO_STEPS {
        let (p, dp) = c.d1(s);
        let uv = surf.project(p);
        let slope = surf.normal(uv.x, uv.y).dot(dp);
        if slope.abs() <= tol::TANGENT * dp.norm() {
            return None;
        }
        let next = (s - signed_distance(surf, p) / slope).clamp(rc.0, rc.1);
        let still = (next - s).abs() <= STILL * (1.0 + s.abs());
        s = next;
        if still {
            break;
        }
    }
    (signed_distance(surf, c.eval(s)).abs() <= SETTLED * tol::LINEAR).then_some(s)
}

/// A span of a curve's range, with a box that holds the curve over it.
struct Span {
    lo: f64,
    hi: f64,
    bounds: Aabb,
}

/// A curve's range cut into spans, each with a box that holds it, for nearest-point queries that
/// only project onto the spans that can hold the nearest point.
struct CurveIndex<'a> {
    curve: &'a Curve,
    range: (f64, f64),
    spans: Vec<Span>,
}

impl<'a> CurveIndex<'a> {
    fn new(curve: &'a Curve, range: (f64, f64)) -> CurveIndex<'a> {
        let mut cuts = with_ends(curve.breakpoints(range.0, range.1), range);
        if cuts.len() < INDEX_SPANS + 1 {
            cuts = evenly_through(range.0, range.1, 2 * INDEX_SPANS).collect();
        }
        let spans = spans(&cuts).map(|(lo, hi)| CurveIndex::span(curve, lo, hi)).collect();
        CurveIndex { curve, range, spans }
    }

    /// The span from `lo` to `hi`: the box of eight equal steps' samples, padded by the longest
    /// chord, since between samples the curve strays from them by less than a chord.
    fn span(curve: &Curve, lo: f64, hi: f64) -> Span {
        let pts: Vec<Point3> = evenly_through(lo, hi, 8).map(|t| curve.eval(t)).collect();
        let mut bounds = Aabb::EMPTY;
        let mut chord: f64 = 0.0;
        if let Some(&first) = pts.first() {
            bounds.add(first);
        }
        for pair in pts.windows(2) {
            if let &[a, b] = pair {
                bounds.add(b);
                chord = chord.max(b.dist(a));
            }
        }
        let pad = Vec3::new(chord, chord, chord);
        bounds.min -= pad;
        bounds.max += pad;
        Span { lo, hi, bounds }
    }

    /// As [`curve_distance`].
    #[allow(clippy::panic)]
    fn distance(&self, p: Point3) -> (f64, f64) {
        let gap = |bb: &Aabb| {
            let d = Vec3::new(
                (bb.min.x - p.x).max(p.x - bb.max.x).max(0.0),
                (bb.min.y - p.y).max(p.y - bb.max.y).max(0.0),
                (bb.min.z - p.z).max(p.z - bb.max.z).max(0.0),
            );
            d.norm()
        };
        let gaps: Vec<(f64, &Span)> = self.spans.iter().map(|s| (gap(&s.bounds), s)).collect();
        // The search stops at the first span whose gap passes the best distance, and the span of
        // least gap comes first, so no span whose gap passes that span's distance is reached.
        let (Some(&(_, nearest)), Some(first)) =
            (gaps.iter().min_by(|a, b| a.0.total_cmp(&b.0)), self.spans.first())
        else {
            panic!("an index over {:?} has spans", self.range);
        };
        let bound = curve_distance(self.curve, (nearest.lo, nearest.hi), p).0;
        let mut order: Vec<(f64, &Span)> = gaps.into_iter().filter(|&(g, _)| g <= bound).collect();
        order.sort_by(|a, b| a.0.total_cmp(&b.0));
        let mut best = (f64::INFINITY, first.lo);
        for (g, span) in order {
            if g > best.0 {
                break;
            }
            let r = curve_distance(self.curve, (span.lo, span.hi), p);
            if r.0 < best.0 {
                best = r;
            }
        }
        best
    }
}

/// The search behind [`curve_curve`] where no closed form applies: the distance to `c2` sampled
/// along `c1`, each local least settled by golden-section search. With `prune`, the distance is
/// only sampled about the pieces of `c1` whose hull boxes come within the tolerance of a hull box
/// of `c2`'s: a least anywhere else is farther than the tolerance and finds nothing, so the answer
/// is the same.
pub(super) fn sampled_curve_curve(
    c1: &Curve,
    r1: (f64, f64),
    c2: &Curve,
    r2: (f64, f64),
    prune: bool,
) -> Vec<Point3> {
    let cuts = with_ends(c1.breakpoints(r1.0, r1.1), r1);
    let mut ts: Vec<f64> = Vec::with_capacity(SPAN_SAMPLES * (cuts.len() - 1) + 1);
    for (a, b) in spans(&cuts) {
        ts.extend(evenly(a, b, SPAN_SAMPLES));
    }
    ts.extend(cuts.last());
    let n = ts.len();
    // Twice the tolerance, so rounding in the boxes can never drop a meeting.
    let reach = prune.then(|| hulls::near(c1, r1, c2, r2, 2.0 * tol::LINEAR)).flatten();
    let wanted = match reach {
        Some(reach) if reach.is_empty() => return Vec::new(),
        Some(reach) => near_samples(&ts, &reach),
        // Every sample, unpruned or where the curves have no hulls.
        None => vec![true; n],
    };
    let index = CurveIndex::new(c2, r2);
    let mut samples: Vec<Option<f64>> = vec![None; n];
    for (k, _) in wanted.iter().enumerate().filter(|&(_, &w)| w) {
        let around = k.saturating_sub(1)..(k + 2).min(n);
        if let (Some(slots), Some(params)) = (samples.get_mut(around.clone()), ts.get(around)) {
            for (slot, &t) in slots.iter_mut().zip(params) {
                if slot.is_none() {
                    *slot = Some(index.distance(c1.eval(t)).0);
                }
            }
        }
    }
    // Each wanted sample's parameter and distance; every neighbour of one was sampled above.
    #[allow(clippy::panic)]
    let value = |k: usize| match (ts.get(k), samples.get(k)) {
        (Some(&t), Some(&Some(d))) => Some((t, d)),
        _ => panic!("a wanted sample's neighbours are sampled"),
    };
    let mut out: Vec<Point3> = Vec::new();
    for (k, _) in wanted.iter().enumerate().filter(|&(_, &w)| w) {
        let Some(at) = value(k) else { continue };
        let lo = if k > 0 { value(k - 1) } else { Some(at) };
        let hi = if k + 1 < n { value(k + 1) } else { Some(at) };
        let (Some(lo), Some(hi)) = (lo, hi) else { continue };
        if at.1 > lo.1 || at.1 > hi.1 {
            continue;
        }
        // Inside a stretch where the curves run together: the stretch's ends are vertices.
        if (k > 0 && lo.1 <= tol::LINEAR) || (k + 1 < n && hi.1 <= tol::LINEAR) {
            continue;
        }
        let t = minimise(&mut |t: f64| index.distance(c1.eval(t)).0, lo.0, hi.0);
        let p = c1.eval(t);
        let (d, s) = index.distance(p);
        if d <= tol::LINEAR {
            let q = p.lerp(c2.eval(s), 0.5);
            if !out.iter().any(|o| o.dist(q) <= tol::LINEAR) {
                out.push(q);
            }
        }
    }
    out
}

/// Which of the samples `ts`, in order, the curve–curve search may find a meeting at: every one
/// whose neighbours' span reaches one of the intervals `reach`, in order.
fn near_samples(ts: &[f64], reach: &[(f64, f64)]) -> Vec<bool> {
    let n = ts.len();
    let at = |k: usize| ts.get(k).copied().unwrap_or(f64::NAN);
    let mut want = vec![false; n];
    let mut k = 0;
    for &(lo, hi) in reach {
        while k < n && at((k + 1).min(n - 1)) < lo {
            k += 1;
        }
        let mut j = k;
        while j < n && at(j.saturating_sub(1)) <= hi {
            if let Some(w) = want.get_mut(j) {
                *w = true;
            }
            j += 1;
        }
    }
    want
}
