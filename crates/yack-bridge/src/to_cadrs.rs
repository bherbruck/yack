// yack: springrep `Brep` → cadrs `Body` (MAPPING.md §1–§4, the S→C column).
use std::collections::BTreeSet;

use yack_cadrs::native::pcurve::{closed_periods, pcurve};
use yack_cadrs::native::tol::CONFUSION;
use yack_cadrs::native::{
    Affine, Body, CheckLevel, Coedge, Curve as CCurve, Edge as CEdge, Face as CFace, Frame as CFrame, NurbsCurve as CNurbsCurve,
    NurbsSurface as CNurbsSurface, P3, Shell as CShell, Solid as CSolid, Surface as CSurface, V3, Vertex as CVertex,
};
use yack_cadrs::native::{math::P2, nurbs::H};
use yack_springrep::{
    geom::{Curve as SCurve, Surface as SSurface},
    math::{Frame as SFrame, Vec2, Vec3},
    topo::{Brep, CoedgeId, FaceName, PCurve},
};

use crate::{Error, Result};

/// How far a carried-over pcurve may stray from springrep's own, in parameter units, before the
/// conversion is refused rather than trusted.
const PCURVE_SLACK: f64 = 1e-6; // tol: uv agreement of two engines' pcurves of one exact edge

/// Knots this close (relative) to an end of a B-spline's range are that end.
const KNOT_SNAP: f64 = 1e-12; // tol: knot round-off

/// A pcurve whose surface point stays this close to its edge needs no tolerance beyond cadrs's own.
const GAP_TIGHT: f64 = 1e-8; // tol: well inside cadrs's CONFUSION (1e-7 mm)

/// The most a carried pcurve may stray from its edge, in mm; the edge's cadrs tolerance is raised to
/// hold it, and anything wider is refused.
const GAP_CAP: f64 = 1e-5; // tol: cadrs's LOOP_GAP

/// A surface derivative this short means the surface does not depend on that coordinate (a pole).
const DEGENERATE: f64 = 1e-9; // tol: a pole's vanishing partial derivative

/// How many parameters along each coedge the carried pcurve is checked at.
const SAMPLES: usize = 9;

fn p3(v: Vec3) -> P3 {
    P3::new(v.x, v.y, v.z)
}

fn v3(v: Vec3) -> V3 {
    V3::new(v.x, v.y, v.z)
}

fn frame(f: &SFrame) -> CFrame {
    CFrame { o: p3(f.origin), x: v3(f.x), y: v3(f.y), z: v3(f.z) }
}

fn nurbs_curve(degree: usize, knots: &[f64], ctrl: &[Vec3], weights: Option<&[f64]>) -> Result<CCurve> {
    let cps = ctrl
        .iter()
        .enumerate()
        .map(|(i, c)| {
            let w = weights.and_then(|w| w.get(i).copied()).unwrap_or(1.0);
            H::new(c.x * w, c.y * w, c.z * w, w)
        })
        .collect();
    let n = CNurbsCurve::new(degree, knots.to_vec(), cps).map_err(|e| Error::Rejected(vec![e.to_string()]))?;
    Ok(CCurve::nurbs(clamp(n)?))
}

/// The arc of the ellipse `c + a cos t·x + b sin t·y` over `[t0, t1]` as an exact rational quadratic
/// B-spline on `s ∈ [0, 1]`, in pieces of at most a quarter turn (cadrs's own form of a conic):
/// each piece is the affine image of a circular arc, its middle pole where the end tangents meet,
/// weighted by the cosine of half its angle.
fn ellipse_arc(c: P3, x: V3, y: V3, a: f64, b: f64, (t0, t1): (f64, f64)) -> Result<CNurbsCurve> {
    let n = (((t1 - t0) / std::f64::consts::FRAC_PI_2).ceil() as usize).max(1);
    let at = |phi: f64, scale: f64| c + x * (a * phi.cos() * scale) + y * (b * phi.sin() * scale);
    let mut knots = vec![0.0; 3];
    let mut cps = Vec::with_capacity(2 * n + 1);
    for i in 0..n {
        let (p0, p1) = (t0 + (t1 - t0) * i as f64 / n as f64, t0 + (t1 - t0) * (i + 1) as f64 / n as f64);
        let h = 0.5 * (p1 - p0);
        let w = h.cos();
        if i == 0 {
            let q = at(p0, 1.0);
            cps.push(H::new(q.x, q.y, q.z, 1.0));
        }
        let m = at(p0 + h, 1.0 / w);
        cps.push(H::new(m.x * w, m.y * w, m.z * w, w));
        let q = at(p1, 1.0);
        cps.push(H::new(q.x, q.y, q.z, 1.0));
        let s = (i + 1) as f64 / n as f64;
        if i + 1 < n {
            knots.extend([s, s]);
        }
    }
    knots.extend([1.0; 3]);
    CNurbsCurve::new(2, knots, cps).map_err(|e| Error::Rejected(vec![e.to_string()]))
}

/// Whether `knots` start and end with `p + 1` equal knots.
fn clamped(knots: &[f64], p: usize) -> bool {
    let n = knots.len();
    n >= 2 * (p + 1)
        && knots.get(..=p).is_some_and(|k| k.iter().all(|&x| Some(&x) == knots.first()))
        && knots.get(n - p - 1..).is_some_and(|k| k.iter().all(|&x| Some(&x) == knots.last()))
}

/// The same curve with its end knots repeated `degree + 1` times (cadrs's B-spline code assumes
/// clamped ends): knots a rounding error from an end are made that end, then the end knot is
/// inserted (Boehm, exact) until it is a C0 cut, and the curve outside its range is dropped.
fn clamp(mut n: CNurbsCurve) -> Result<CNurbsCurve> {
    let p = n.degree;
    let (a, b) = n.range();
    let near = |x: f64, k: f64| (x - k).abs() <= KNOT_SNAP * (1.0 + k.abs());
    for k in &mut n.knots {
        if near(*k, a) && *k != a && *k >= a {
            *k = a;
        } else if near(*k, b) && *k != b && *k <= b {
            *k = b;
        }
    }
    if clamped(&n.knots, p) {
        return Ok(n);
    }
    let count = |n: &CNurbsCurve, x: f64| n.knots.iter().filter(|&&k| k == x).count();
    let reject = || Error::Unsupported("a B-spline whose ends cannot be clamped".into());
    if n.knots.first() != Some(&a) || count(&n, a) < p + 1 {
        while count(&n, a) < p {
            n = n.insert_knot(a);
        }
        let s = n.knots.iter().position(|&k| k == a).ok_or_else(reject)?;
        if s >= 1 {
            let mut knots = vec![a];
            knots.extend_from_slice(n.knots.get(s..).ok_or_else(reject)?);
            let cps = n.cps.get(s - 1..).ok_or_else(reject)?.to_vec();
            n = CNurbsCurve { degree: p, knots, cps };
        }
    }
    if n.knots.last() != Some(&b) || count(&n, b) < p + 1 {
        while count(&n, b) < p {
            n = n.insert_knot(b);
        }
        let e = n.knots.iter().rposition(|&k| k == b).ok_or_else(reject)?;
        if e + 1 < n.knots.len() {
            let mut knots = n.knots.get(..=e).ok_or_else(reject)?.to_vec();
            knots.push(b);
            let cps = n.cps.get(..=e.checked_sub(p).ok_or_else(reject)?).ok_or_else(reject)?.to_vec();
            n = CNurbsCurve { degree: p, knots, cps };
        }
    }
    if !clamped(&n.knots, p) || n.knots.len() != n.cps.len() + p + 1 {
        return Err(reject());
    }
    Ok(n)
}

/// A springrep B-spline over `range` as a cadrs NURBS. A periodic one (unclamped knots with
/// `knot(i + n) = knot(i) + period`, control points repeating every `n`) wraps its parameter;
/// cadrs's does not, so it is unrolled over exactly the spans `range` covers: the same curve,
/// knots and control points, shifted by whole periods.
fn bspline(b: &yack_springrep::geom::BSpline, range: (f64, f64)) -> Result<CCurve> {
    let (p, knots, ctrl) = (b.degree(), b.knots(), b.control_points());
    let Some(period) = b.period() else {
        return nurbs_curve(p, knots, ctrl, None);
    };
    let (d0, d1) = b.domain();
    if range.0 >= d0 && range.1 <= d1 {
        return nurbs_curve(p, knots, ctrl, None);
    }
    let n = ctrl.len().saturating_sub(p);
    if n == 0 || knots.len() < n + p + 1 {
        return Err(Error::Unsupported("a periodic B-spline with too few control points".into()));
    }
    let ni = n as i64;
    let knot = |i: i64| -> f64 {
        let (q, r) = (i.div_euclid(ni), i.rem_euclid(ni) as usize);
        knots.get(r).copied().unwrap_or(f64::NAN) + period * q as f64
    };
    // The span `j` (knot(j) ≤ t < knot(j + 1), or ≤ for the right end) holding `t`.
    let span = |t: f64, right: bool| -> i64 {
        let q = ((t - d0) / period).floor() as i64;
        let base = t - period * q as f64;
        let mut j = p as i64;
        while j < (p + n) as i64 - 1 && if right { knot(j + 1) < base } else { knot(j + 1) <= base } {
            j += 1;
        }
        j + q * ni
    };
    let (j0, j1) = (span(range.0, false), span(range.1, true));
    let pi = p as i64;
    let ks: Vec<f64> = (j0 - pi..=j1 + pi + 1).map(knot).collect();
    let cs: Vec<Vec3> = (j0 - pi..=j1)
        .map(|i| ctrl.get(i.rem_euclid(ni) as usize).copied().unwrap_or(Vec3::ZERO))
        .collect();
    // Clamped to the edge's own range by knot insertion (exact), so neither engine sees the
    // unrolled spline's unclamped ends.
    match nurbs_curve(p, &ks, &cs, None)? {
        CCurve::Nurbs(n) => Ok(CCurve::nurbs(n.restricted(range.0, range.1))),
        other => Ok(other),
    }
}

/// A 3-D edge curve, and how far it may lie from springrep's (zero unless approximated).
fn curve(c: &SCurve, range: (f64, f64)) -> Result<(CCurve, f64)> {
    Ok(match c {
        SCurve::Line(l) => (CCurve::Line { o: p3(l.origin), d: v3(l.dir) }, 0.0),
        SCurve::Circle(k) => {
            let f = &k.frame;
            (CCurve::Circle { c: p3(f.origin), x: v3(f.x), y: v3(f.y), r: k.radius }, 0.0)
        }
        SCurve::Ellipse(k) => {
            let f = &k.frame;
            (CCurve::Ellipse { c: p3(f.origin), x: v3(f.x), y: v3(f.y), a: k.major, b: k.minor }, 0.0)
        }
        SCurve::BSpline(b) => (bspline(b, range)?, 0.0),
        SCurve::Nurbs(n) => {
            let (d0, d1) = n.domain();
            let whole = nurbs_curve(n.degree(), n.knots(), n.control_points(), n.weights())?;
            if range.0 >= d0 && range.1 <= d1 {
                (whole, 0.0)
            } else if n.period().is_some() {
                // A closed clamped curve springrep wraps; repeated end to end over the edge's range
                // (each pole a one-point row of the surface case: the same exact concatenation).
                let CCurve::Nurbs(c) = whole else { return Err(Error::Unsupported("a NURBS curve that is not one".into())) };
                let rows: Vec<Vec<H>> = c.cps.iter().map(|h| vec![*h]).collect();
                let (knots, rows) = unroll_u(c.degree, c.knots.clone(), rows, range)?;
                let cps = rows.into_iter().flatten().collect();
                let m = CNurbsCurve::new(c.degree, knots, cps).map_err(|e| Error::Rejected(vec![e.to_string()]))?;
                (CCurve::nurbs(clamp(m)?), 0.0)
            } else {
                return Err(Error::Unsupported("an edge past the ends of an open NURBS curve".into()));
            }
        }
        SCurve::Intersection(ic) => {
            let a = &ic.approx;
            let out = bspline(a, range)?;
            // The approximation springrep's STEP writer exports; measure how far it strays.
            let gap = (0..SAMPLES)
                .map(|i| {
                    let t = range.0 + (range.1 - range.0) * i as f64 / (SAMPLES - 1) as f64;
                    c.eval(t).dist(a.eval(t))
                })
                .fold(0.0, f64::max);
            (out, gap)
        }
    })
}

fn unreachable_conic<T>() -> Result<T> {
    Err(Error::Unsupported("a conic surface without its conic".into()))
}

/// Whether the cadrs form of `s` re-parametrises it (a swept ellipse becomes a swept rational
/// B-spline on [0, 1]), so springrep's pcurves on it do not carry over.
fn reparametrised(s: &SSurface) -> bool {
    match s {
        SSurface::Extrusion(e) => matches!(e.curve, SCurve::Ellipse(_)),
        SSurface::Revolution(r) => matches!(r.curve, SCurve::Ellipse(_)),
        _ => false,
    }
}

/// A pcurve made by cadrs itself for an edge or a face the bridge re-parametrised, and how far its
/// surface point strays from the edge (cadrs's own geometry on both sides).
fn fresh_pcurve(surf: &CSurface, edge: &CEdge, near: Option<P2>) -> Result<(CCurve, f64)> {
    if edge.degenerate {
        // Tried: the pole as a straight line at s = 0 or 1. It converts, but springrep then
        // classifies the body brought back differently (bench spline/rational-cap), so refused.
        return Err(Error::Unsupported("a pole on a re-parametrised conic surface".into()));
    }
    let [t0, t1] = edge.t;
    let pc = pcurve(surf, &edge.curve, t0, t1, near);
    let gap = (0..2 * SAMPLES - 1)
        .map(|i| {
            let t = t0 + (t1 - t0) * i as f64 / (2 * SAMPLES - 2) as f64;
            let uv = pc.point(t);
            (surf.eval(uv.x, uv.y) - edge.curve.point(t)).norm()
        })
        .fold(0.0, f64::max);
    if !(gap <= GAP_CAP) {
        return Err(Error::Unsupported(format!("cadrs's pcurve of a re-parametrised conic strays {gap:.1e} mm")));
    }
    Ok((pc, gap))
}

/// The pcurve of an ellipse edge carried over as a rational B-spline on `s ∈ [0, 1]`: springrep's
/// own pcurve read at the angle `t(s)` of each new parameter's point, interpolated in `s` (finer
/// until it holds to its edge), so it stays on springrep's branch of a closed surface.
fn remapped_pcurve(
    b: &Brep,
    c: CoedgeId,
    el: &yack_springrep::geom::Ellipse,
    surf: &CSurface,
    edge: &CEdge,
) -> Result<(CCurve, f64)> {
    let (t0, t1) = b.edge(b.coedge(c).edge).range;
    let f = &el.frame;
    // The angle of the ellipse's point at s, on the branch nearest the straight guess.
    let angle = |s: f64| {
        let p = edge.curve.point(s);
        let d = Vec3::new(p.x, p.y, p.z) - f.origin;
        let raw = (d.dot(f.y) / el.minor).atan2(d.dot(f.x) / el.major);
        let guess = t0 + (t1 - t0) * s;
        raw + ((guess - raw) / std::f64::consts::TAU).round() * std::f64::consts::TAU
    };
    let mut best: Option<(CCurve, f64)> = None;
    for n in [33usize, 129, 513] {
        let ss = spaced(0.0, 1.0, n);
        let ts: Vec<f64> = ss.iter().map(|&s| angle(s).clamp(t0, t1)).collect();
        let uv = b.coedge_uvs(c, &ts);
        let pts: Vec<P3> = uv.iter().map(|w| P3::new(w.x, w.y, 0.0)).collect();
        let fit = CCurve::nurbs(CNurbsCurve::interpolate(&pts, &ss, 3, None, None));
        let mids: Vec<f64> = ss.windows(2).map(|w| 0.5 * (w[0] + w[1])).collect();
        let gap = ss
            .iter()
            .chain(&mids)
            .map(|&s| {
                let q = fit.point(s);
                (surf.eval(q.x, q.y) - edge.curve.point(s)).norm()
            })
            .fold(0.0, f64::max);
        if best.as_ref().is_none_or(|(_, g)| gap < *g) {
            best = Some((fit, gap));
        }
        if gap <= GAP_TIGHT {
            break;
        }
    }
    match best {
        Some((pc, g)) if g <= GAP_CAP => Ok((pc, g)),
        other => Err(Error::Unsupported(format!(
            "an ellipse edge's pcurve strays {:?} mm once re-parametrised",
            other.map(|(_, g)| g)
        ))),
    }
}

/// A range widened to hold `want` (a face's span of one parameter).
fn widen(domain: (f64, f64), want: (f64, f64)) -> (f64, f64) {
    (domain.0.min(want.0), domain.1.max(want.1))
}

/// A clamped closed net (its first and last rows along u coincide) repeated end to end so it spans
/// `want` in u: springrep wraps a closed surface's parameter, cadrs evaluates only inside the knots.
/// The copies meet with C0 knots; the surface is the same point set, parameter for parameter.
pub(crate) fn unroll_u(p: usize, knots: Vec<f64>, rows: Vec<Vec<H>>, want: (f64, f64)) -> Result<(Vec<f64>, Vec<Vec<H>>)> {
    let (Some(&a), Some(&b)) = (knots.first(), knots.last()) else {
        return Err(Error::Unsupported("a B-spline net with no knots".into()));
    };
    let period = b - a;
    if !(period > 0.0) || (want.0 >= a && want.1 <= b) {
        return Ok((knots, rows));
    }
    let q0 = ((want.0 - a) / period).floor() as i64;
    let q1 = ((want.1 - a) / period).ceil() as i64 - 1;
    if q1 < q0 || q1 - q0 > 16 {
        return Err(Error::Unsupported("a face spanning too many periods of a closed spline".into()));
    }
    let interior = knots.get(p + 1..knots.len().saturating_sub(p + 1)).unwrap_or(&[]).to_vec();
    let shift = |q: i64| period * q as f64;
    let mut k = vec![a + shift(q0); p + 1];
    let mut out_rows = Vec::new();
    for q in q0..=q1 {
        if q > q0 {
            k.extend(std::iter::repeat_n(a + shift(q), p));
        }
        k.extend(interior.iter().map(|x| x + shift(q)));
        let skip = usize::from(q > q0);
        out_rows.extend(rows.iter().skip(skip).cloned());
    }
    k.extend(std::iter::repeat_n(b + shift(q1), p + 1));
    Ok((k, out_rows))
}

/// `s`, a closed B-spline surface, unrolled over the uv span the face's loops reach; any other
/// surface as it is.
fn cover_loops(s: CSurface, edges: &[CEdge], loops: &[Vec<Coedge>], extra: [Option<f64>; 2]) -> Result<CSurface> {
    let CSurface::Nurbs(n) = &s else { return Ok(s) };
    let base = closed_periods(&s);
    let periods = [base[0].or(extra[0]), base[1].or(extra[1])];
    if periods.iter().all(Option::is_none) {
        return Ok(s);
    }
    let mut span = ((f64::INFINITY, f64::NEG_INFINITY), (f64::INFINITY, f64::NEG_INFINITY));
    for co in loops.iter().flatten() {
        let Some(e) = edges.get(co.edge) else { continue };
        for t in spaced(e.t[0], e.t[1], SAMPLES) {
            let w = co.pc.point(t);
            span = ((span.0.0.min(w.x), span.0.1.max(w.x)), (span.1.0.min(w.y), span.1.1.max(w.y)));
        }
    }
    let ((u0, u1), (v0, v1)) = n.range();
    let slack = PCURVE_SLACK;
    let inside_u = span.0.0 >= u0 - slack && span.0.1 <= u1 + slack;
    let inside_v = span.1.0 >= v0 - slack && span.1.1 <= v1 + slack;
    if inside_u && inside_v {
        return Ok(s);
    }
    let (mut ku, mut kv, mut cps) = (n.ku.clone(), n.kv.clone(), n.cps.clone());
    if !inside_u && periods[0].is_some() {
        (ku, cps) = unroll_u(n.du, ku, cps, span.0)?;
    }
    if !inside_v && periods[1].is_some() {
        let (k, rows) = unroll_u(n.dv, kv, transpose(cps), span.1)?;
        kv = k;
        cps = transpose(rows);
    }
    let m = CNurbsSurface::new(n.du, n.dv, ku, kv, cps).map_err(|e| Error::Rejected(vec![e.to_string()]))?;
    Ok(CSurface::Nurbs(std::sync::Arc::new(m)))
}

pub(crate) fn transpose(rows: Vec<Vec<H>>) -> Vec<Vec<H>> {
    let n = rows.first().map_or(0, Vec::len);
    (0..n).map(|j| rows.iter().filter_map(|r| r.get(j).copied()).collect()).collect()
}

/// The cadrs surface of `s` for a face spanning `uv` (u range, v range) of its parameters.
fn surface(s: &SSurface, uv: ((f64, f64), (f64, f64))) -> Result<CSurface> {
    Ok(match s {
        SSurface::Plane(p) => CSurface::Plane { f: frame(&p.frame) },
        SSurface::Cylinder(c) => CSurface::Cylinder { f: frame(&c.frame), r: c.radius },
        SSurface::Cone(c) => CSurface::Cone { f: frame(&c.frame), r: c.radius, a: c.half_angle },
        SSurface::Sphere(c) => CSurface::Sphere { f: frame(&c.frame), r: c.radius },
        SSurface::Torus(c) => CSurface::Torus { f: frame(&c.frame), rmaj: c.major, rmin: c.minor },
        SSurface::Extrusion(e) if matches!(e.curve, SCurve::Ellipse(_)) => {
            let SCurve::Ellipse(el) = &e.curve else { return unreachable_conic(); };
            let f = &el.frame;
            let arc = ellipse_arc(p3(f.origin), v3(f.x), v3(f.y), el.major, el.minor, uv.0)?;
            CSurface::Extrusion { c: CCurve::nurbs(arc), d: v3(e.dir) }
        }
        SSurface::Revolution(r) if matches!(r.curve, SCurve::Ellipse(_)) => {
            let SCurve::Ellipse(el) = &r.curve else { return unreachable_conic(); };
            let f = &el.frame;
            let arc = ellipse_arc(p3(f.origin), v3(f.x), v3(f.y), el.major, el.minor, uv.1)?;
            let m = CCurve::nurbs(arc);
            let mid = r.curve.eval(0.5 * (uv.1.0 + uv.1.1));
            let fr = CFrame::from_xz(p3(r.origin), v3(mid - r.origin), v3(r.axis));
            CSurface::Revolution { f: fr, m }
        }
        SSurface::Extrusion(e) => {
            let (c, gap) = curve(&e.curve, widen(e.curve.domain(), uv.0))?;
            if gap > 0.0 {
                return Err(Error::Unsupported("an extrusion of an intersection curve".into()));
            }
            CSurface::Extrusion { c, d: v3(e.dir) }
        }
        SSurface::Revolution(r) => {
            let (m, gap) = curve(&r.curve, widen(r.curve.domain(), uv.1))?;
            if gap > 0.0 {
                return Err(Error::Unsupported("a revolution of an intersection curve".into()));
            }
            let (t0, t1) = r.curve.domain();
            let mid = r.curve.eval(0.5 * (t0 + t1));
            let f = CFrame::from_xz(p3(r.origin), v3(mid - r.origin), v3(r.axis));
            CSurface::Revolution { f, m }
        }
        SSurface::Ruled(_) => return Err(Error::Unsupported("a ruled surface (MAPPING.md §1: not yet)".into())),
        SSurface::Nurbs(n) => {
            let (du, dv) = n.degrees();
            let (ku, kv) = n.knots();
            // cadrs assumes clamped nets; springrep's are (a closed one closes by its boundary rows).
            if !clamped(ku, du) || !clamped(kv, dv) {
                return Err(Error::Unsupported("an unclamped B-spline surface".into()));
            }
            let nu = ku.len().saturating_sub(du + 1);
            let nv = kv.len().saturating_sub(dv + 1);
            let ctrl = n.control_points();
            let weights = n.weights();
            let mut cps = Vec::with_capacity(nu);
            for i in 0..nu {
                let mut row = Vec::with_capacity(nv);
                for j in 0..nv {
                    let k = i * nv + j;
                    let c = ctrl.get(k).ok_or_else(|| Error::Unsupported("a NURBS net of the wrong size".into()))?;
                    let w = weights.and_then(|w| w.get(k).copied()).unwrap_or(1.0);
                    row.push(H::new(c.x * w, c.y * w, c.z * w, w));
                }
                cps.push(row);
            }
            let (mut ku, mut kv) = (ku.to_vec(), kv.to_vec());
            if n.u_period().is_some() {
                (ku, cps) = unroll_u(du, ku, cps, uv.0)?;
            }
            if n.v_period().is_some() {
                let (k, rows) = unroll_u(dv, kv, transpose(cps), uv.1)?;
                kv = k;
                cps = transpose(rows);
            }
            let n = CNurbsSurface::new(du, dv, ku, kv, cps)
                .map_err(|e| Error::Rejected(vec![e.to_string()]))?;
            CSurface::Nurbs(std::sync::Arc::new(n))
        }
    })
}

/// The surface in cadrs's native orientation, and whether its v was negated to get there. cadrs's
/// ops assume an analytic surface's natural normal points away from its axis; springrep may give
/// a cone a half-angle past ±90° (v running toward the apex), whose natural normal points in, and
/// reverse the face. That cone is the same set of points with v → −v and a → a ∓ π.
fn canonical(s: CSurface) -> (CSurface, bool, f64) {
    let (s, flipped) = match s {
        CSurface::Cone { f, r, a } if a.cos() < 0.0 => {
            let a = if a > 0.0 { a - std::f64::consts::PI } else { a + std::f64::consts::PI };
            (CSurface::Cone { f, r, a }, true)
        }
        other => (other, false),
    };
    // cadrs's ops place their seams by the frame's x, which its constructors take from the axis
    // alone (`Frame::from_z`); a springrep frame turned about the axis from that one gives cadrs a
    // seam where its own sweeps and booleans would not put it (bench swept/ring-pushed-in-past-its-
    // fold: the cut came out with a face its STEP reads as another region). The same surface in
    // cadrs's frame is the old one at u − φ.
    let turn = |f: CFrame| -> (CFrame, f64) {
        let native = CFrame::from_z(f.o, f.z);
        let phi = native.x.dot(&f.y).atan2(native.x.dot(&f.x));
        (CFrame { o: f.o, x: native.x, y: f.z.cross(&native.x), z: f.z }, phi)
    };
    let (s, du) = match s {
        CSurface::Cylinder { f, r } => {
            let (f, phi) = turn(f);
            (CSurface::Cylinder { f, r }, -phi)
        }
        CSurface::Cone { f, r, a } => {
            let (f, phi) = turn(f);
            (CSurface::Cone { f, r, a }, -phi)
        }
        CSurface::Sphere { f, r } => {
            let (f, phi) = turn(f);
            (CSurface::Sphere { f, r }, -phi)
        }
        CSurface::Torus { f, rmaj, rmin } => {
            let (f, phi) = turn(f);
            (CSurface::Torus { f, rmaj, rmin }, -phi)
        }
        other => (other, 0.0),
    };
    (s, flipped, du)
}

/// How near, relative, a cone face's v span must end to the apex for the face to be drawn from it.
const AT_APEX: f64 = 1e-6; // tol: a face's uv span against the apex's parameter

/// A cone face that reaches its apex, drawn from the apex as cadrs draws its own pointed cones
/// (radius 0 at v = 0, v growing away from it): cadrs sweeps a pointed cone exactly only in that
/// form (bench swept/cone-spun-about-a-line-through-it). Takes the canonical surface and the
/// face's canonical v span; gives the new surface and the uv map `(u, v) → (s·u + cu, s·v + cv)`.
/// With sin a > 0 the apex is at v = −r/sin a below the face: the frame slides there (s = +1).
/// With sin a < 0 it is above: the axis turns over and both parameters run back from the apex
/// (s = −1), a map of determinant +1, so the natural normal, and the face's sense, stay.
fn from_apex(s: &CSurface, v_span: (f64, f64)) -> Option<(CSurface, f64, f64, f64)> {
    let CSurface::Cone { f, r, a } = s else { return None };
    let (sin, cos) = a.sin_cos();
    // A cone already drawn from its apex (r = 0, opening along +v) needs nothing; springrep's
    // pointed cone is r = 0 too but runs v ≤ 0 with sin a < 0, the second case below.
    if sin == 0.0 || (*r == 0.0 && sin > 0.0) {
        return None;
    }
    let va = -r / sin;
    let near = |v: f64| (v - va).abs() <= AT_APEX * va.abs().max(1.0);
    let o = f.o + f.z * (va * cos);
    if sin > 0.0 && near(v_span.0) {
        let f2 = CFrame { o, x: f.x, y: f.y, z: f.z };
        Some((CSurface::Cone { f: f2, r: 0.0, a: *a }, 1.0, 0.0, -va))
    } else if sin < 0.0 && near(v_span.1) {
        let z = -f.z;
        let native = CFrame::from_z(o, z);
        let phi = native.x.dot(&f.y).atan2(native.x.dot(&f.x));
        let f2 = CFrame { o, x: native.x, y: z.cross(&native.x), z };
        Some((CSurface::Cone { f: f2, r: 0.0, a: -a }, -1.0, phi, va))
    } else {
        None
    }
}

/// The pcurve under the uv map `(u, v) → (s·u + cu, s·v + cv)`, `s` ±1 (see [`from_apex`]).
fn mapped_uv(pc: CCurve, s: f64, cu: f64, cv: f64) -> CCurve {
    let pt = |p: P3| P3::new(s * p.x + cu, s * p.y + cv, p.z);
    let dir = |v: V3| V3::new(s * v.x, s * v.y, v.z);
    match pc {
        CCurve::Line { o, d } => CCurve::Line { o: pt(o), d: dir(d) },
        CCurve::Circle { c, x, y, r } => CCurve::Circle { c: pt(c), x: dir(x), y: dir(y), r },
        CCurve::Ellipse { c, x, y, a, b } => CCurve::Ellipse { c: pt(c), x: dir(x), y: dir(y), a, b },
        CCurve::Nurbs(n) => {
            let mut n = (*n).clone();
            for h in &mut n.cps {
                (h.x, h.y) = (s * h.x + cu * h.w, s * h.y + cv * h.w);
            }
            CCurve::Nurbs(std::sync::Arc::new(n))
        }
    }
}

/// The pcurve with its v negated (for a surface [`canonical`] flipped).
fn flip_v(pc: CCurve) -> CCurve {
    let neg = |p: P3| P3::new(p.x, -p.y, p.z);
    let negv = |v: V3| V3::new(v.x, -v.y, v.z);
    match pc {
        CCurve::Line { o, d } => CCurve::Line { o: neg(o), d: negv(d) },
        CCurve::Circle { c, x, y, r } => CCurve::Circle { c: neg(c), x: negv(x), y: negv(y), r },
        CCurve::Ellipse { c, x, y, a, b } => CCurve::Ellipse { c: neg(c), x: negv(x), y: negv(y), a, b },
        CCurve::Nurbs(n) => {
            let mut n = (*n).clone();
            for h in &mut n.cps {
                h.y = -h.y;
            }
            CCurve::Nurbs(std::sync::Arc::new(n))
        }
    }
}

/// A straight pcurve from `a` at `t0` to `b` at `t1`, parametrised by the edge's t.
fn straight(a: Vec2, b: Vec2, t0: f64, t1: f64) -> Result<CCurve> {
    let cps = vec![H::new(a.x, a.y, 0.0, 1.0), H::new(b.x, b.y, 0.0, 1.0)];
    let n = CNurbsCurve::new(1, vec![t0, t0, t1, t1], cps).map_err(|e| Error::Rejected(vec![e.to_string()]))?;
    Ok(CCurve::nurbs(n))
}

fn spaced(t0: f64, t1: f64, n: usize) -> Vec<f64> {
    (0..n).map(|i| t0 + (t1 - t0) * i as f64 / (n - 1) as f64).collect()
}

fn params(t0: f64, t1: f64) -> Vec<f64> {
    spaced(t0, t1, SAMPLES)
}

/// The coedge's pcurve in cadrs's form: by the edge's t, on the branch springrep's runs on.
fn coedge_pcurve(b: &Brep, c: CoedgeId, surf: &CSurface, edge_curve: Option<&CCurve>) -> Result<(CCurve, f64)> {
    let co = b.coedge(c);
    let e = b.edge(co.edge);
    let s = &b.face(b.coedge_face(c)).surface;
    let (t0, t1) = e.range;
    let ts = params(t0, t1);
    let want = b.coedge_uvs(c, &ts);
    let (Some(&first), Some(&last)) = (want.first(), want.last()) else {
        return Err(Error::Unsupported(format!("coedge {} has no pcurve samples", c.index())));
    };
    // The edge's own point at t (its vertex for a pole).
    let on_edge = |t: f64| match &e.curve {
        Some(k) => k.eval(t),
        None => b.vertex(e.start).point,
    };
    // How far a candidate's uv strays from springrep's, in parameter units, leaving out a
    // coordinate the surface does not depend on there (u at a pole), where any value is right.
    let uv_off = |pc: &CCurve, t: f64, w: Vec2| -> f64 {
        let g = pc.point(t);
        let (_, su, sv) = s.d1(w.x, w.y);
        let du = if su.norm() <= DEGENERATE { 0.0 } else { (g.x - w.x).abs() };
        let dv = if sv.norm() <= DEGENERATE { 0.0 } else { (g.y - w.y).abs() };
        du.max(dv) / (1.0 + w.x.abs().max(w.y.abs()))
    };
    // How far the candidate's surface point strays from the edge, in mm.
    let gap = |pc: &CCurve, t: f64| -> f64 {
        let g = pc.point(t);
        s.eval(g.x, g.y).dist(on_edge(t))
    };
    // Checked at the samples and halfway between them.
    // In increasing order: springrep follows a periodic pcurve from one parameter to the next.
    let mut check_ts = ts.clone();
    check_ts.extend(ts.windows(2).map(|w| 0.5 * (w[0] + w[1])));
    check_ts.sort_by(f64::total_cmp);
    let check_uv = b.coedge_uvs(c, &check_ts);
    let judge = |pc: &CCurve| -> Option<f64> {
        let mut worst = 0.0f64;
        for (&t, &w) in check_ts.iter().zip(&check_uv) {
            let d = uv_off(pc, t, w);
            if d.is_nan() || d > PCURVE_SLACK {
                return None;
            }
            worst = worst.max(gap(pc, t));
        }
        worst.is_finite().then_some(worst)
    };
    let first_try = match (&co.pcurve, edge_curve) {
        (PCurve::Affine { .. }, _) | (_, None) => straight(first, last, t0, t1)?,
        (PCurve::Projected { .. }, Some(curve)) => {
            let mid = want.get(SAMPLES / 2).copied().unwrap_or(first);
            pcurve(surf, curve, t0, t1, Some(P2::new(mid.x, mid.y)))
        }
    };
    let mut best: Option<(CCurve, f64)> = judge(&first_try).map(|g| (first_try.clone(), g));
    if best.as_ref().is_some_and(|(_, g)| *g <= GAP_TIGHT) {
        return best.ok_or_else(|| Error::Unsupported("unreachable".into()));
    }
    // cadrs's own fit went astray or is loose (it can wrap a closed ring at its seam): interpolate
    // springrep's pcurve itself, in the edge's t, finer until it is tight.
    for n in [33usize, 129, 513, 2049] {
        let ts_n = spaced(t0, t1, n);
        let uv = b.coedge_uvs(c, &ts_n);
        let pts: Vec<P3> = uv.iter().map(|w| P3::new(w.x, w.y, 0.0)).collect();
        let fit = CCurve::nurbs(CNurbsCurve::interpolate(&pts, &ts_n, 3, None, None));
        let mids: Vec<f64> = ts_n.windows(2).map(|w| 0.5 * (w[0] + w[1])).collect();
        let fit_gap = mids.iter().map(|&t| gap(&fit, t)).fold(0.0, f64::max);
        if let Some(g) = judge(&fit).map(|g| g.max(fit_gap)) {
            if best.as_ref().is_none_or(|(_, b)| g < *b) {
                best = Some((fit, g));
            }
            if g <= GAP_TIGHT {
                break;
            }
        }
    }
    match best {
        Some((pc, g)) if g <= GAP_CAP => Ok((pc, g)),
        other => {
            let curve = e.curve.as_ref().map_or("degenerate", |c| c.kind());
            let along: Vec<String> = ts
                .iter()
                .zip(&want)
                .map(|(&t, w)| {
                    let g = first_try.point(t);
                    format!("t {t:.4}: sr ({:.4}, {:.4}) cadrs ({:.4}, {:.4})", w.x, w.y, g.x, g.y)
                })
                .collect();
            Err(Error::Unsupported(format!(
                "coedge {} ({curve} on {}): no pcurve within {GAP_CAP:e} mm (best {:?}): {}",
                c.index(),
                s.kind(),
                other.map(|(_, g)| g),
                along.join("; ")
            )))
        }
    }
}

/// Shifts each coedge's pcurve by whole periods so it starts where the one before it ends: springrep
/// keeps each coedge on its own branch of a periodic surface, cadrs needs the loop continuous in uv.
fn chain(surf: &CSurface, edges: &[CEdge], loop_: &mut [Coedge], extra: [Option<f64>; 2]) {
    let base = closed_periods(surf);
    let periods = [base[0].or(extra[0]), base[1].or(extra[1])];
    if periods.iter().all(Option::is_none) {
        return;
    }
    let ends = |c: &Coedge| {
        let t = edges.get(c.edge).map_or([0.0, 0.0], |e| e.t);
        if c.fwd { (t[0], t[1]) } else { (t[1], t[0]) }
    };
    let whole = |gap: f64, period: Option<f64>| period.map_or(0.0, |p| (gap / p).round() * p);
    for k in 1..loop_.len() {
        let (Some(prev), Some(next)) = (loop_.get(k - 1), loop_.get(k)) else { continue };
        let end = prev.pc.point(ends(prev).1);
        let start = next.pc.point(ends(next).0);
        let (du, dv) = (whole(end.x - start.x, periods[0]), whole(end.y - start.y, periods[1]));
        if (du, dv) != (0.0, 0.0)
            && let Some(c) = loop_.get_mut(k)
        {
            c.pc = c.pc.transformed(&Affine::translation(V3::new(du, dv, 0.0)));
        }
    }
}

/// The loop with a degenerate edge put in wherever it jumps in one parameter at a point where the
/// surface does not depend on that parameter (a pole, an apex): springrep lets a loop's coedges
/// meet a pole at different `u`, cadrs wants the jump carried by a degenerate edge. New edges go
/// after every existing one, so no index moves.
fn bridge_poles(surf: &CSurface, edges: &mut Vec<CEdge>, lp: Vec<Coedge>) -> Vec<Coedge> {
    let ends = |c: &Coedge, edges: &[CEdge]| {
        let t = edges.get(c.edge).map_or([0.0, 0.0], |e| e.t);
        let (ts, te) = if c.fwd { (t[0], t[1]) } else { (t[1], t[0]) };
        (c.pc.point(ts), c.pc.point(te))
    };
    let end_vertex = |c: &Coedge, edges: &[CEdge]| edges.get(c.edge).map(|e| if c.fwd { e.v[1] } else { e.v[0] });
    let n = lp.len();
    let mut out = Vec::with_capacity(n + 2);
    for k in 0..n {
        let Some(cur) = lp.get(k) else { continue };
        out.push(cur.clone());
        let Some(next) = lp.get((k + 1) % n) else { continue };
        let (_, a) = ends(cur, edges);
        let (b, _) = ends(next, edges);
        let (du, dv) = (b.x - a.x, b.y - a.y);
        // Which parameter jumps, and whether the surface ignores it there.
        // A small step both ways (one of them may leave a clamped surface's knots, where cadrs holds
        // the end point): a pole moves neither.
        let ignores = |along_u: bool| {
            let here = surf.eval(a.x, a.y);
            [-POLE_STEP, POLE_STEP].iter().all(|&h| {
                let q = if along_u { surf.eval(a.x + h, a.y) } else { surf.eval(a.x, a.y + h) };
                (q - here).norm() <= POLE_SPAN
            })
        };
        let jump = if du.abs() > PCURVE_SLACK && dv.abs() <= PCURVE_SLACK && ignores(true) {
            Some((true, a.x, b.x, a.y))
        } else if dv.abs() > PCURVE_SLACK && du.abs() <= PCURVE_SLACK && ignores(false) {
            Some((false, a.y, b.y, a.x))
        } else {
            None
        };
        let (Some((along_u, from, to, fixed)), Some(v)) = (jump, end_vertex(cur, edges)) else { continue };
        let Some(at) = edges.get(cur.edge).map(|e| if cur.fwd { e.curve.point(e.t[1]) } else { e.curve.point(e.t[0]) }) else {
            continue;
        };
        let (t0, t1) = (from.min(to), from.max(to));
        let (pa, pb) = if along_u { (Vec2::new(t0, fixed), Vec2::new(t1, fixed)) } else { (Vec2::new(fixed, t0), Vec2::new(fixed, t1)) };
        let Ok(pc) = straight(pa, pb, t0, t1) else { continue };
        edges.push(CEdge { curve: CCurve::Line { o: at, d: V3::zeros() }, t: [t0, t1], v: [v, v], degenerate: true, tol: CONFUSION });
        out.push(Coedge { edge: edges.len() - 1, fwd: to > from, pc });
    }
    out
}

/// A surface whose points `POLE_STEP` of a parameter apart, both ways, are within `POLE_SPAN` does
/// not depend on that parameter there.
const POLE_STEP: f64 = 1e-3; // tol: a parameter step well inside any face
const POLE_SPAN: f64 = 1e-10; // tol: a pole's span, mm

/// The cadrs body of `b`, and each face's springrep names (by face index) to restore on the way
/// back.
pub fn to_cadrs(b: &Brep) -> Result<(Body, Vec<BTreeSet<FaceName>>)> {
    // Built field by field (yack-cadrs's `testkit` makes them public) so that every entity keeps
    // its index and its tolerance, then checked by cadrs's full checker.
    let mut body = Body::default();
    for v in b.vertex_ids() {
        body.verts.push(CVertex { p: p3(b.vertex(v).point), tol: b.vertex_tolerance(v).max(CONFUSION) });
    }
    // Edges re-parametrised on the way (their pcurves are made afresh by cadrs).
    let mut as_nurbs: Vec<usize> = Vec::new();
    for e in b.edge_ids() {
        if b.edge_uses(e).len() > 2 {
            return Err(Error::Unsupported(format!("edge {} is used by more than two faces", e.index())));
        }
        let edge = b.edge(e);
        let v = [edge.start.index(), edge.end.index()];
        let t = [edge.range.0, edge.range.1];
        let tol = b.edge_tolerance(e).max(CONFUSION);
        body.edges.push(match &edge.curve {
            None => CEdge {
                curve: CCurve::Line { o: p3(b.vertex(edge.start).point), d: V3::zeros() },
                t,
                v,
                degenerate: true,
                tol,
            },
            Some(SCurve::Ellipse(el)) => {
                // cadrs's ops expect a conic as its rational B-spline (MAPPING.md §2).
                let f = &el.frame;
                let n = ellipse_arc(p3(f.origin), v3(f.x), v3(f.y), el.major, el.minor, edge.range)?;
                as_nurbs.push(e.index());
                CEdge { curve: CCurve::nurbs(n), t: [0.0, 1.0], v, degenerate: false, tol }
            }
            Some(c) => {
                let (curve, gap) = curve(c, edge.range)?;
                CEdge { curve, t, v, degenerate: false, tol: tol.max(2.0 * gap) }
            }
        });
    }
    let mut names = Vec::new();
    for f in b.face_ids() {
        let face = b.face(f);
        if face.shell.is_none() {
            return Err(Error::Unsupported(format!("face {} belongs to no shell", f.index())));
        }
        // Pcurves are made in springrep's parametrisation (the same formula as cadrs's for this
        // surface), then carried onto the canonical one.
        // The face's span of each parameter, from its loops (springrep's own branch).
        let mut uv = ((f64::INFINITY, f64::NEG_INFINITY), (f64::INFINITY, f64::NEG_INFINITY));
        for &lp in &face.loops {
            for &c in &b.lp(lp).coedges {
                let (t0, t1) = b.edge(b.coedge(c).edge).range;
                for w in b.coedge_uvs(c, &params(t0, t1)) {
                    uv = ((uv.0.0.min(w.x), uv.0.1.max(w.x)), (uv.1.0.min(w.y), uv.1.1.max(w.y)));
                }
            }
        }
        let surf = surface(&face.surface, uv)?;
        let fresh_face = reparametrised(&face.surface);
        let (canon, flipped, turned_u) = canonical(surf.clone());
        // A cone face reaching its apex is drawn from it (its canonical v span is springrep's,
        // negated where canonical flipped it).
        let v_span = if flipped { (-uv.1.1, -uv.1.0) } else { uv.1 };
        let apex = from_apex(&canon, v_span);
        let canon = apex.as_ref().map_or(canon, |a| a.0.clone());
        // Periods cadrs may not see for itself. A whole swept ellipse re-parametrised onto
        // s ∈ [0, 1] closes with period 1 there; springrep knows when its spline surface closes (to
        // its tolerance).
        let closes = |span: (f64, f64)| span.1 - span.0 >= std::f64::consts::TAU - PCURVE_SLACK;
        let extra_periods = match &face.surface {
            SSurface::Extrusion(_) if fresh_face && closes(uv.0) => [Some(1.0), None],
            SSurface::Revolution(_) if fresh_face && closes(uv.1) => [None, Some(1.0)],
            SSurface::Nurbs(n) => [n.u_period(), n.v_period()],
            _ => [None, None],
        };
        let mut loops = Vec::with_capacity(face.loops.len());
        for &lp in &face.loops {
            let mut coedges = Vec::new();
            for &c in &b.lp(lp).coedges {
                let co = b.coedge(c);
                let curve = body.edges.get(co.edge.index()).filter(|e| !e.degenerate).map(|e| e.curve.clone());
                let (pc, gap) = if fresh_face || as_nurbs.contains(&co.edge.index()) {
                    let Some(edge) = body.edges.get(co.edge.index()) else {
                        return Err(Error::Unsupported(format!("edge {} out of range", co.edge.index())));
                    };
                    // springrep's uv at the edge's middle places a periodic surface's branch.
                    let near = (!fresh_face).then(|| {
                        let (t0, t1) = b.edge(co.edge).range;
                        let w = b.coedge_uvs(c, &[0.5 * (t0 + t1)]);
                        w.first().map(|w| P2::new(w.x, w.y))
                    });
                    match (fresh_face, b.edge(co.edge).curve.as_ref()) {
                        // Only the edge was re-parametrised: follow springrep's own pcurve (its
                        // branch of a closed surface) at the angle each new parameter stands for.
                        (false, Some(SCurve::Ellipse(el))) => remapped_pcurve(b, c, el, &surf, edge)?,
                        _ => fresh_pcurve(&surf, edge, near.flatten())?,
                    }
                } else {
                    coedge_pcurve(b, c, &surf, curve.as_ref())?
                };
                if gap > 0.0 {
                    let mut ends = [None, None];
                    if let Some(edge) = body.edges.get_mut(co.edge.index()) {
                        edge.tol = edge.tol.max(2.0 * gap);
                        ends = [Some(edge.v[0]), Some(edge.v[1])];
                    }
                    for v in ends.into_iter().flatten() {
                        if let Some(vx) = body.verts.get_mut(v) {
                            vx.tol = vx.tol.max(2.0 * gap);
                        }
                    }
                }
                let pc = if flipped { flip_v(pc) } else { pc };
                let pc = if turned_u != 0.0 { pc.transformed(&Affine::translation(V3::new(turned_u, 0.0, 0.0))) } else { pc };
                let pc = match &apex {
                    Some((_, s, cu, cv)) => mapped_uv(pc, *s, *cu, *cv),
                    None => pc,
                };
                coedges.push(Coedge { edge: co.edge.index(), fwd: !co.reversed, pc });
            }
            chain(&canon, &body.edges, &mut coedges, extra_periods);
            let coedges = bridge_poles(&canon, &mut body.edges, coedges);
            loops.push(coedges);
        }
        // A closed spline surface the chained loops now run past (springrep's loop jumps a period
        // at the seam; cadrs's runs on): unroll it over where they go, so they stay on it.
        let canon = cover_loops(canon, &body.edges, &loops, extra_periods)?;
        body.faces.push(CFace { surf: canon, sense: face.reversed == flipped, loops });
        names.push(face.names.clone());
    }
    for s in b.solid_ids() {
        let solid = b.solid(s);
        if solid.outer != 1 {
            return Err(Error::Unsupported(format!("solid {} has {} outer shells", s.index(), solid.outer)));
        }
        let mut shells = Vec::new();
        for &sh in &solid.shells {
            body.shells.push(CShell { faces: b.shell(sh).faces.iter().map(|f| f.index()).collect(), closed: true });
            shells.push(body.shells.len() - 1);
        }
        body.solids.push(CSolid { shells });
    }
    let mut issues = body.check_with(CheckLevel::Full);
    if !issues.is_empty() {
        // Which springrep surfaces the faces cadrs refused came from, to say what to fix.
        let named: BTreeSet<usize> = issues
            .iter()
            .filter_map(|m| m.strip_prefix("face ").and_then(|r| r.split(' ').next()).and_then(|n| n.parse().ok()))
            .collect();
        for f in named.into_iter().take(3) {
            if let Some(face) = b.face_ids().nth(f).map(|id| b.face(id)) {
                let kinds: Vec<&str> = face
                    .loops
                    .iter()
                    .flat_map(|&lp| b.lp(lp).coedges.iter())
                    .map(|&c| b.edge(b.coedge(c).edge).curve.as_ref().map_or("pole", |k| k.kind()))
                    .collect();
                let periods = match &face.surface {
                    SSurface::Nurbs(n) => format!(" periods u {:?} v {:?} domain {:?}", n.u_period(), n.v_period(), n.domain()),
                    _ => String::new(),
                };
                let cadrs_periods = body.faces.get(f).map(|cf| format!(" cadrs periods {:?}", closed_periods(&cf.surf))).unwrap_or_default();
                issues.push(format!(
                    "(face {f} was springrep {}{periods}{cadrs_periods}; its edges {})",
                    face.surface.kind(),
                    kinds.join(",")
                ));
            }
        }
        return Err(Error::Rejected(issues));
    }
    Ok((body, names))
}
