// yack: a STEP edge's own pcurves (its SURFACE_CURVE's PCURVEs), read and checked against it.
//! The pcurves a file gives an edge, where its curve is a `SURFACE_CURVE` or `SEAM_CURVE`: each
//! `PCURVE`'s surface record and its 2-D curve (a `LINE` or a B-spline in a
//! `DEFINITIONAL_REPRESENTATION`), as a kernel [`PCurve`] in the kernel curve's own parameter.
//! A given pcurve is only a claim: [`strays`] measures how far it leaves the edge's curve and its
//! surface's parameter range, at equal parameters, so a reader takes it only where it holds and
//! projects the edge everywhere else.
// Step 1 (measure-only): only the reader's probe calls these until topology takes the pcurves.
#![cfg_attr(not(test), allow(dead_code))]

use yack_p21::{Entity, Param};

use super::geometry::Geometry;
use super::part21;
use crate::{
    Error, Result,
    geom::{Curve, NurbsCurve, Surface},
    math::{Point3, Vec2},
    topo::PCurve,
};

fn bad<T>(id: u64, what: &str) -> Result<T> {
    Err(Error::InvalidInput(format!("STEP #{id}: {what}")))
}

/// A pcurve the file gives an edge's curve on one surface.
#[derive(Clone, Debug, PartialEq)]
pub(super) struct Given {
    /// The surface record its `PCURVE` names (a face takes it when it names the same one).
    pub surface: u64,
    /// The pcurve, in the kernel curve's parameter.
    pub pcurve: PCurve,
}

/// The kernel curve's parameter per file parameter of `curve` (record `id`): the kernel measures a
/// line in millimetres where the file measures it in its `VECTOR`'s lengths, and keeps the file's
/// parameter for every other curve it reads (an angle for a conic; a knot for a B-spline). `None`
/// for a curve whose parameter the reader does not keep.
fn scale_of(geo: &Geometry, id: u64, curve: &Curve) -> Result<Option<f64>> {
    match curve {
        Curve::Line(_) => {
            let Some(line) = entity(geo, id)?.part("LINE") else { return bad(id, "expected LINE") };
            let v = geo.reference(id, line.get(2))?;
            let Some(vector) = entity(geo, v)?.part("VECTOR") else {
                return bad(v, "expected VECTOR");
            };
            Ok(Some(geo.real(v, vector.get(2))? * geo.length_unit()))
        }
        Curve::Circle(_) | Curve::Ellipse(_) | Curve::Nurbs(_) => Ok(Some(1.0)),
        Curve::BSpline(_) | Curve::Intersection(_) => Ok(None),
    }
}

fn entity<'a>(geo: &Geometry<'a>, id: u64) -> Result<&'a Entity> {
    match geo.file.get(id) {
        Some(e) => Ok(e),
        None => bad(id, "no such instance"),
    }
}

/// The pcurves of an `EDGE_CURVE`'s geometry record `id`, and how many of its `PCURVE`s are in a
/// form this reader does not take (a 2-D curve other than a line or a B-spline, a curve whose
/// parameter it does not keep). None and none for a curve that is not a `SURFACE_CURVE` or
/// `SEAM_CURVE`.
pub(super) fn given(geo: &mut Geometry, id: u64) -> Result<(Vec<Given>, usize)> {
    let e = entity(geo, id)?;
    let Some(a) = e.part("SURFACE_CURVE").or_else(|| e.part("SEAM_CURVE")) else {
        return Ok((Vec::new(), 0));
    };
    let c3 = geo.reference(id, a.get(1))?;
    let curve = geo.curve(c3)?;
    let refs = references(a.get(2));
    let Some(k) = scale_of(geo, c3, &curve)? else {
        return Ok((Vec::new(), refs.len()));
    };
    let (mut out, mut odd) = (Vec::new(), 0);
    for r in refs {
        match pcurve(geo, r, k)? {
            Some(g) => out.push(g),
            None => odd += 1,
        }
    }
    Ok((out, odd))
}

/// `PCURVE` record `id` with its file parameter `k` times the kernel's; `None` when it is not a
/// `PCURVE` (another associated geometry) or its 2-D curve is in another form.
fn pcurve(geo: &Geometry, id: u64, k: f64) -> Result<Option<Given>> {
    let Some(a) = entity(geo, id)?.part("PCURVE") else { return Ok(None) };
    let surface = geo.reference(id, a.get(1))?;
    let rep = geo.reference(id, a.get(2))?;
    let Some(r) = entity(geo, rep)?.part("DEFINITIONAL_REPRESENTATION") else { return Ok(None) };
    let &[c2] = references(r.get(1)).as_slice() else { return Ok(None) };
    Ok(curve_2d(geo, c2, k)?.map(|pcurve| Given { surface, pcurve }))
}

/// The instances a list parameter names, in order.
fn references(p: Option<&Param>) -> Vec<u64> {
    p.and_then(Param::list).unwrap_or(&[]).iter().filter_map(Param::reference).collect()
}

/// Two finite numbers: a 2-D point or direction.
fn pair(geo: &Geometry, id: u64, p: Option<&Param>) -> Result<Vec2> {
    match *geo.numbers(id, p)?.as_slice() {
        [x, y] if x.is_finite() && y.is_finite() => Ok(Vec2::new(x, y)),
        _ => bad(id, "expected two finite coordinates"),
    }
}

/// A 2-D `CARTESIAN_POINT` (parameters, never scaled to millimetres).
fn point_2d(geo: &Geometry, id: u64) -> Result<Vec2> {
    match entity(geo, id)?.part("CARTESIAN_POINT") {
        Some(a) => pair(geo, id, a.get(1)),
        None => bad(id, "expected a 2-D CARTESIAN_POINT"),
    }
}

/// 2-D curve record `id` as a pcurve in the kernel parameter t = s / k of its file parameter s.
fn curve_2d(geo: &Geometry, id: u64, k: f64) -> Result<Option<PCurve>> {
    let e = entity(geo, id)?;
    if let Some(a) = e.part("LINE") {
        let p = point_2d(geo, geo.reference(id, a.get(1))?)?;
        let v = geo.reference(id, a.get(2))?;
        let Some(va) = entity(geo, v)?.part("VECTOR") else { return bad(v, "expected VECTOR") };
        let d = geo.reference(v, va.get(1))?;
        let Some(da) = entity(geo, d)?.part("DIRECTION") else {
            return bad(d, "expected DIRECTION");
        };
        let (dir, len) = (pair(geo, d, da.get(1))?, geo.real(v, va.get(2))?);
        let n = dir.norm();
        if n <= 0.0 || !n.is_finite() || !len.is_finite() || k == 0.0 {
            return bad(id, "a degenerate 2-D line");
        }
        // uv(s) = p + s·len·dir/|dir| with s = t·k
        return Ok(Some(PCurve::Affine { origin: p, dir: dir * (len * k / n) }));
    }
    if e.is("B_SPLINE_CURVE_WITH_KNOTS") {
        return Ok(bspline_2d(geo, id, e, k)?.map(|c| PCurve::Fitted { curve: Box::new(c) }));
    }
    Ok(None)
}

/// A 2-D B-spline as a kernel curve through (u, v, 0) in t = s / k; `None` where its parts make no
/// curve (the reader then projects).
fn bspline_2d(geo: &Geometry, id: u64, e: &Entity, k: f64) -> Result<Option<NurbsCurve>> {
    let Some(w) = e.part("B_SPLINE_CURVE_WITH_KNOTS") else { return Ok(None) };
    let (head, knots): (&[Param], &[Param]) = match e.part("B_SPLINE_CURVE") {
        Some(h) => (h, w),
        None => match (w.get(1..6), w.get(6..)) {
            (Some(h), Some(rest)) if w.len() >= 9 => (h, rest),
            _ => return bad(id, "short B_SPLINE_CURVE_WITH_KNOTS"),
        },
    };
    let degree = match head.first().and_then(part21::integral) {
        Some(d) if d > 0 => d as usize,
        _ => return bad(id, "bad B-spline degree"),
    };
    let mut ctrl = Vec::new();
    for p in head.get(1).and_then(Param::list).unwrap_or(&[]) {
        let uv = point_2d(geo, geo.reference(id, Some(p))?)?;
        ctrl.push(Point3::new(uv.x, uv.y, 0.0));
    }
    let (m, s) = (geo.numbers(id, knots.first())?, geo.numbers(id, knots.get(1))?);
    if m.len() != s.len() || k == 0.0 {
        return bad(id, "knot and multiplicity counts differ");
    }
    let mut kv = Vec::new();
    for (&mi, &si) in m.iter().zip(&s) {
        if mi < 0.0 || mi.fract() != 0.0 {
            return bad(id, "bad knot multiplicity");
        }
        kv.extend(std::iter::repeat_n(si / k, mi as usize));
    }
    let weights = match e.part("RATIONAL_B_SPLINE_CURVE") {
        Some(r) => Some(geo.numbers(id, r.first())?),
        None => None,
    };
    Ok(NurbsCurve::new(degree, kv, ctrl, weights).ok())
}

/// Equal steps a given pcurve is sampled at over its edge, and more per knot span of a fitted one.
const SAMPLES: usize = 256;
const PER_SPAN: usize = 8;

/// How far `pc` on `s` strays from `c` over `t0..t1`: the most |S(pc(t)) − C(t)| at equal `t`,
/// and the most it leaves the surface's parameter range (a B-spline's knots; never along a
/// periodic direction). `None` for a projected pcurve, which gives no parameters.
pub(super) fn strays(
    s: &Surface,
    c: &Curve,
    pc: &PCurve,
    (t0, t1): (f64, f64),
) -> Option<(f64, f64)> {
    let at = |i: usize, n: usize| t0 + (t1 - t0) * i as f64 / n as f64;
    let mut ts: Vec<f64> = (0..=SAMPLES).map(|i| at(i, SAMPLES)).collect();
    if let PCurve::Fitted { curve } = pc {
        let inside = |k: &f64| *k > t0 && *k < t1;
        let mut knots: Vec<f64> = curve.knots().iter().copied().filter(inside).collect();
        knots.dedup();
        let mut cuts = vec![t0];
        cuts.extend(knots);
        cuts.push(t1);
        for (a, b) in cuts.iter().zip(cuts.iter().skip(1)) {
            ts.extend((1..PER_SPAN).map(|i| a + (b - a) * i as f64 / PER_SPAN as f64));
        }
    }
    let range = match s {
        Surface::Nurbs(n) => {
            let ((u0, u1), (v0, v1)) = n.domain();
            let free = |p: Option<f64>| p.is_some();
            Some((u0, u1, v0, v1, free(n.u_period()), free(n.v_period())))
        }
        _ => None,
    };
    let (mut off, mut outside) = (0.0_f64, 0.0_f64);
    for t in ts {
        let uv = pc.uv_at(t)?;
        off = off.max(s.eval(uv.x, uv.y).dist(c.eval(t)));
        if let Some((u0, u1, v0, v1, pu, pv)) = range {
            let du = if pu { 0.0 } else { (u0 - uv.x).max(uv.x - u1) };
            let dv = if pv { 0.0 } else { (v0 - uv.y).max(uv.y - v1) };
            outside = outside.max(du).max(dv);
        }
    }
    Some((off, outside))
}
