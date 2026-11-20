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
    Ok(CCurve::nurbs(n))
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
            if range.0 < d0 || range.1 > d1 {
                return Err(Error::Unsupported("an edge past the ends of a closed NURBS curve".into()));
            }
            (nurbs_curve(n.degree(), n.knots(), n.control_points(), n.weights())?, 0.0)
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

fn surface(s: &SSurface) -> Result<CSurface> {
    Ok(match s {
        SSurface::Plane(p) => CSurface::Plane { f: frame(&p.frame) },
        SSurface::Cylinder(c) => CSurface::Cylinder { f: frame(&c.frame), r: c.radius },
        SSurface::Cone(c) => CSurface::Cone { f: frame(&c.frame), r: c.radius, a: c.half_angle },
        SSurface::Sphere(c) => CSurface::Sphere { f: frame(&c.frame), r: c.radius },
        SSurface::Torus(c) => CSurface::Torus { f: frame(&c.frame), rmaj: c.major, rmin: c.minor },
        SSurface::Extrusion(e) => {
            let (c, gap) = curve(&e.curve, e.curve.domain())?;
            if gap > 0.0 {
                return Err(Error::Unsupported("an extrusion of an intersection curve".into()));
            }
            CSurface::Extrusion { c, d: v3(e.dir) }
        }
        SSurface::Revolution(r) => {
            let (m, gap) = curve(&r.curve, r.curve.domain())?;
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
            let n = CNurbsSurface::new(du, dv, ku.to_vec(), kv.to_vec(), cps)
                .map_err(|e| Error::Rejected(vec![e.to_string()]))?;
            CSurface::Nurbs(std::sync::Arc::new(n))
        }
    })
}

/// The surface in cadrs's native orientation, and whether its v was negated to get there. cadrs's
/// ops assume an analytic surface's natural normal points away from its axis; springrep may give
/// a cone a half-angle past ±90° (v running toward the apex), whose natural normal points in, and
/// reverse the face. That cone is the same set of points with v → −v and a → a ∓ π.
fn canonical(s: CSurface) -> (CSurface, bool) {
    match s {
        CSurface::Cone { f, r, a } if a.cos() < 0.0 => {
            let a = if a > 0.0 { a - std::f64::consts::PI } else { a + std::f64::consts::PI };
            (CSurface::Cone { f, r, a }, true)
        }
        other => (other, false),
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
fn chain(surf: &CSurface, edges: &[CEdge], loop_: &mut [Coedge]) {
    let periods = closed_periods(surf);
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

/// The cadrs body of `b`, and each face's springrep names (by face index) to restore on the way
/// back.
pub fn to_cadrs(b: &Brep) -> Result<(Body, Vec<BTreeSet<FaceName>>)> {
    // Built field by field (yack-cadrs's `testkit` makes them public) so that every entity keeps
    // its index and its tolerance, then checked by cadrs's full checker.
    let mut body = Body::default();
    for v in b.vertex_ids() {
        body.verts.push(CVertex { p: p3(b.vertex(v).point), tol: b.vertex_tolerance(v).max(CONFUSION) });
    }
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
        let surf = surface(&face.surface)?;
        let (canon, flipped) = canonical(surf.clone());
        let mut loops = Vec::with_capacity(face.loops.len());
        for &lp in &face.loops {
            let mut coedges = Vec::new();
            for &c in &b.lp(lp).coedges {
                let co = b.coedge(c);
                let curve = body.edges.get(co.edge.index()).filter(|e| !e.degenerate).map(|e| e.curve.clone());
                let (pc, gap) = coedge_pcurve(b, c, &surf, curve.as_ref())?;
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
                coedges.push(Coedge { edge: co.edge.index(), fwd: !co.reversed, pc });
            }
            chain(&canon, &body.edges, &mut coedges);
            loops.push(coedges);
        }
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
    let issues = body.check_with(CheckLevel::Full);
    if !issues.is_empty() {
        return Err(Error::Rejected(issues));
    }
    Ok((body, names))
}
