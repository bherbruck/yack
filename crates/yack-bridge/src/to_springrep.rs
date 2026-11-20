// yack: cadrs `Body` → springrep `Brep` (MAPPING.md §1–§4, the C→S column).
use std::collections::BTreeSet;

use yack_cadrs::native::pcurve::closed_periods;
use yack_cadrs::native::{Body, Curve as CCurve, Frame as CFrame, NurbsCurve as CNurbsCurve, P3, Surface as CSurface, V3};
use yack_springrep::{
    facade::bridge::Builder,
    geom::{
        BSpline, Circle, Cone, Curve as SCurve, Cylinder, Ellipse, Extrusion, Line, NurbsCurve as SNurbsCurve,
        NurbsSurface as SNurbsSurface, Plane, Revolution, Sphere, Surface as SSurface, Torus,
    },
    math::{Frame as SFrame, Vec2, Vec3},
    topo::{Brep, CoedgeSpec, EdgeId, FaceName, PCurve, VertexId},
};

use crate::{Error, Result};

/// How close, as a fraction of the period, a coedge must stay to a whole number of periods to lie
/// on the seam line.
const SEAM_SLACK: f64 = 1e-9; // tol: parameter round-off on a seam, relative to the period

/// springrep's `BSpline` stops at cubics; higher degrees go to its `NurbsCurve` (up to 15).
const BSPLINE_MAX_DEGREE: usize = 3;

/// Samples per coedge when measuring how far cadrs's pcurves, curves and vertices stray apart.
const GAP_SAMPLES: usize = 9;

/// How far an edge's 3-D curve and its pcurves on its faces, and its ends and its vertices, lie
/// apart in `body` (cadrs's geometry may meet only to its own tolerance, ~1e-7 mm): springrep
/// carries that as the edge's and vertices' local tolerance rather than refusing the body.
fn gaps(body: &Body) -> (Vec<f64>, Vec<f64>) {
    let mut edge = vec![0.0f64; body.edges().len()];
    let mut vert = vec![0.0f64; body.verts().len()];
    for f in body.faces() {
        for co in f.loops.iter().flatten() {
            let Some(e) = body.edges().get(co.edge) else { continue };
            if e.degenerate {
                continue;
            }
            let worst = (0..GAP_SAMPLES)
                .map(|i| {
                    let t = e.t[0] + (e.t[1] - e.t[0]) * i as f64 / (GAP_SAMPLES - 1) as f64;
                    let uv = co.pc.point(t);
                    (f.surf.eval(uv.x, uv.y) - e.curve.point(t)).norm()
                })
                .fold(0.0, f64::max);
            if let Some(g) = edge.get_mut(co.edge) {
                *g = g.max(worst);
            }
        }
    }
    for e in body.edges() {
        if e.degenerate {
            continue;
        }
        for (k, &t) in e.t.iter().enumerate() {
            let Some(&vi) = e.v.get(k) else { continue };
            if let (Some(vx), Some(g)) = (body.verts().get(vi), vert.get_mut(vi)) {
                *g = g.max((e.curve.point(t) - vx.p).norm());
            }
        }
    }
    (edge, vert)
}

/// How far from 1 a weight may be for a B-spline to count as non-rational.
const UNIT_WEIGHT: f64 = 1e-15; // tol: weights equal to 1 up to rounding

fn v(p: P3) -> Vec3 {
    Vec3::new(p.x, p.y, p.z)
}

fn d(p: V3) -> Vec3 {
    Vec3::new(p.x, p.y, p.z)
}

fn frame(f: &CFrame) -> SFrame {
    SFrame { origin: v(f.o), x: d(f.x), y: d(f.y), z: d(f.z) }
}

fn nurbs_curve(n: &CNurbsCurve) -> Result<SCurve> {
    let ctrl: Vec<Vec3> = n.cps.iter().map(|h| Vec3::new(h.x / h.w, h.y / h.w, h.z / h.w)).collect();
    let rational = n.cps.iter().any(|h| (h.w - 1.0).abs() > UNIT_WEIGHT);
    let reject = |e: yack_springrep::Error| Error::Rejected(vec![e.to_string()]);
    if rational || n.degree > BSPLINE_MAX_DEGREE {
        let w = rational.then(|| n.cps.iter().map(|h| h.w).collect());
        Ok(SCurve::Nurbs(Box::new(SNurbsCurve::new(n.degree, n.knots.clone(), ctrl, w).map_err(reject)?)))
    } else {
        Ok(SCurve::BSpline(BSpline::new(n.degree, n.knots.clone(), ctrl).map_err(reject)?))
    }
}

fn curve(c: &CCurve) -> Result<SCurve> {
    let plane = |c: P3, x: V3, y: V3| SFrame { origin: v(c), x: d(x), y: d(y), z: d(x.cross(&y)) };
    Ok(match c {
        CCurve::Line { o, d: dir } => SCurve::Line(Line { origin: v(*o), dir: d(*dir) }),
        CCurve::Circle { c, x, y, r } => SCurve::Circle(Circle { frame: plane(*c, *x, *y), radius: *r }),
        CCurve::Ellipse { c, x, y, a, b } => {
            SCurve::Ellipse(Ellipse { frame: plane(*c, *x, *y), major: *a, minor: *b })
        }
        CCurve::Nurbs(n) => nurbs_curve(n)?,
    })
}

fn surface(s: &CSurface) -> Result<SSurface> {
    Ok(match s {
        CSurface::Plane { f } => SSurface::Plane(Plane { frame: frame(f) }),
        CSurface::Cylinder { f, r } => SSurface::Cylinder(Cylinder { frame: frame(f), radius: *r }),
        CSurface::Cone { f, r, a } => SSurface::Cone(Cone { frame: frame(f), radius: *r, half_angle: *a }),
        CSurface::Sphere { f, r } => SSurface::Sphere(Sphere { frame: frame(f), radius: *r }),
        CSurface::Torus { f, rmaj, rmin } => SSurface::Torus(Torus { frame: frame(f), major: *rmaj, minor: *rmin }),
        CSurface::Extrusion { c, d: dir } => SSurface::Extrusion(Box::new(Extrusion { curve: curve(c)?, dir: d(*dir) })),
        CSurface::Revolution { f, m } => {
            SSurface::Revolution(Box::new(Revolution { curve: curve(m)?, origin: v(f.o), axis: d(f.z) }))
        }
        CSurface::Nurbs(n) => {
            let nu = n.cps.len();
            let nv = n.cps.first().map_or(0, Vec::len);
            let mut ctrl = Vec::with_capacity(nu * nv);
            let mut w = Vec::with_capacity(nu * nv);
            for row in &n.cps {
                if row.len() != nv {
                    return Err(Error::Unsupported("a NURBS surface with ragged control rows".into()));
                }
                for h in row {
                    ctrl.push(Vec3::new(h.x / h.w, h.y / h.w, h.z / h.w));
                    w.push(h.w);
                }
            }
            let rational = w.iter().any(|x| (x - 1.0).abs() > UNIT_WEIGHT);
            let s = SNurbsSurface::new(
                (n.du, n.dv),
                (n.ku.clone(), n.kv.clone()),
                (nu, nv),
                ctrl,
                rational.then_some(w),
            )
            .map_err(|e| Error::Rejected(vec![e.to_string()]))?;
            SSurface::Nurbs(Box::new(s))
        }
    })
}

/// The pcurve in springrep's form: `Affine` where cadrs's is straight in the edge's t, else
/// `Projected` from where the coedge starts.
fn pcurve(pc: &CCurve, t: [f64; 2], fwd: bool) -> PCurve {
    let straight = match pc {
        CCurve::Line { o, d } => Some((Vec2::new(o.x, o.y), Vec2::new(d.x, d.y))),
        CCurve::Nurbs(n) if n.degree == 1 && n.cps.len() == 2 && n.cps.iter().all(|h| h.w == 1.0) => {
            match (n.cps.first(), n.cps.last(), n.knots.get(1), n.knots.get(2)) {
                (Some(a), Some(b), Some(&k0), Some(&k1)) if k1 > k0 => {
                    let dir = Vec2::new((b.x - a.x) / (k1 - k0), (b.y - a.y) / (k1 - k0));
                    Some((Vec2::new(a.x - dir.x * k0, a.y - dir.y * k0), dir))
                }
                _ => None,
            }
        }
        _ => None,
    };
    match straight {
        Some((origin, dir)) => PCurve::Affine { origin, dir },
        None => {
            let p = pc.point(if fwd { t[0] } else { t[1] });
            PCurve::Projected { seed: Vec2::new(p.x, p.y) }
        }
    }
}

/// One coedge's pcurve with its uv at the start, middle and end of the coedge.
struct Uses {
    pc: PCurve,
    start: Vec2,
    mid: Vec2,
    end: Vec2,
}

/// Moves a loop's pcurves, all by one whole number of periods, so the loop starts in the surface's
/// base period: cadrs keeps a loop continuous in uv but may run it a period on (its loops are
/// chained from wherever the first coedge sits), springrep wants the same continuous loop with its
/// lowest parameter in `[0, period)`, so that a seam lies at 0 and at one period.
fn normalise(lp: &mut [Uses], periods: [Option<f64>; 2]) {
    for (axis, period) in periods.into_iter().enumerate() {
        let Some(p) = period.filter(|p| *p > 0.0) else { continue };
        let at = |w: Vec2| if axis == 0 { w.x } else { w.y };
        let low = lp.iter().flat_map(|u| [at(u.start), at(u.mid), at(u.end)]).fold(f64::INFINITY, f64::min);
        if !low.is_finite() {
            continue;
        }
        let s = -((low + SEAM_SLACK * p) / p).floor() * p;
        if s == 0.0 {
            continue;
        }
        let d = if axis == 0 { Vec2::new(s, 0.0) } else { Vec2::new(0.0, s) };
        let add = |w: Vec2| Vec2::new(w.x + d.x, w.y + d.y);
        for u in lp.iter_mut() {
            (u.start, u.mid, u.end) = (add(u.start), add(u.mid), add(u.end));
            u.pc = match u.pc {
                PCurve::Affine { origin, dir } => PCurve::Affine { origin: add(origin), dir },
                PCurve::Projected { seed } => PCurve::Projected { seed: add(seed) },
            };
        }
    }
}

/// The springrep `Brep` of `body`, giving face `i` the names `names[i]` (none past its end).
/// Vertices, edges and faces keep their indices. A body with an edge no face uses is refused
/// here, since dropping it would renumber the edges after it; [`to_springrep_mapped`] takes it.
pub fn to_springrep(body: &Body, names: &[BTreeSet<FaceName>]) -> Result<Brep> {
    let (b, map) = to_springrep_mapped(body, names)?;
    // Index for index, or not at all: a caller mapping ids by index must not see edges move.
    match map.iter().enumerate().find(|&(i, m)| *m != Some(i)) {
        None => Ok(b),
        Some((i, _)) => Err(Error::Unsupported(format!(
            "cadrs edge {i} is used by no face; dropping it renumbers edges (use to_springrep_mapped)"
        ))),
    }
}

/// [`to_springrep`], with where each cadrs edge went: `map[i]` is cadrs edge `i`'s springrep
/// edge index, `None` for an edge no face used (dropped). The identity when none was dropped.
pub fn to_springrep_mapped(body: &Body, names: &[BTreeSet<FaceName>]) -> Result<(Brep, Vec<Option<usize>>)> {
    if !body.wires().is_empty() || !body.points().is_empty() {
        return Err(Error::Unsupported("a body with wires or acorn points".into()));
    }
    if body.shells().iter().any(|s| !s.closed) {
        return Err(Error::Unsupported("a body with an open shell (a sheet)".into()));
    }
    let mut bb = Builder::new();
    let (edge_gap, vert_gap) = gaps(body);
    // Twice the measured gap: the tolerance must hold the gap, not just reach it.
    let held = |tol: f64, gap: Option<&f64>| tol.max(2.0 * gap.copied().unwrap_or(0.0));
    let verts: Vec<VertexId> =
        body.verts().iter().enumerate().map(|(i, vx)| bb.vertex(v(vx.p), held(vx.tol, vert_gap.get(i)))).collect();
    let vert = |i: usize| verts.get(i).copied().ok_or_else(|| Error::Unsupported(format!("vertex {i} out of range")));
    // An edge no face uses (cadrs leaves some after an op) is dropped; the map says where each
    // cadrs edge went.
    let mut used = vec![false; body.edges().len()];
    for co in body.faces().iter().flat_map(|f| f.loops.iter().flatten()) {
        if let Some(u) = used.get_mut(co.edge) {
            *u = true;
        }
    }
    let mut edges: Vec<Option<EdgeId>> = Vec::with_capacity(body.edges().len());
    for (i, e) in body.edges().iter().enumerate() {
        if !used.get(i).copied().unwrap_or(false) {
            edges.push(None);
            continue;
        }
        let range = (e.t[0], e.t[1]);
        edges.push(Some(if e.degenerate {
            bb.degenerate_edge(vert(e.v[0])?, range)
        } else {
            bb.edge(curve(&e.curve)?, range, vert(e.v[0])?, vert(e.v[1])?, held(e.tol, edge_gap.get(i)))
        }));
    }
    let mut faces = Vec::with_capacity(body.faces().len());
    for (i, f) in body.faces().iter().enumerate() {
        let id = bb.face(surface(&f.surf)?, !f.sense, names.get(i).cloned().unwrap_or_default().into_iter().collect());
        let periods = closed_periods(&f.surf);
        for lp in &f.loops {
            let mut ids = Vec::with_capacity(lp.len());
            let mut uses = Vec::with_capacity(lp.len());
            for co in lp {
                let edge = edges
                    .get(co.edge)
                    .copied()
                    .flatten()
                    .ok_or_else(|| Error::Unsupported(format!("edge {} out of range", co.edge)))?;
                let t = body.edges().get(co.edge).map_or([0.0, 0.0], |e| e.t);
                let (ts, te) = if co.fwd { (t[0], t[1]) } else { (t[1], t[0]) };
                let uv = |x: f64| {
                    let p = co.pc.point(x);
                    Vec2::new(p.x, p.y)
                };
                uses.push(Uses { pc: pcurve(&co.pc, t, co.fwd), start: uv(ts), mid: uv(0.5 * (ts + te)), end: uv(te) });
                ids.push((edge, !co.fwd));
            }
            normalise(&mut uses, periods);
            let specs = ids
                .into_iter()
                .zip(uses)
                .map(|((edge, reversed), u)| CoedgeSpec { edge, reversed, pcurve: u.pc })
                .collect();
            bb.add_loop(id, specs);
        }
        faces.push(id);
    }
    let mut shells = Vec::with_capacity(body.shells().len());
    for s in body.shells() {
        let fs = s.faces.iter().map(|&f| faces.get(f).copied().ok_or_else(|| Error::Unsupported(format!("face {f} out of range"))));
        shells.push(bb.shell(fs.collect::<Result<_>>()?));
    }
    for s in body.solids() {
        let sh = s.shells.iter().map(|&i| shells.get(i).copied().ok_or_else(|| Error::Unsupported(format!("shell {i} out of range"))));
        bb.solid(sh.collect::<Result<_>>()?, 1);
    }
    let map = edges.iter().map(|e| e.map(EdgeId::index)).collect();
    let b = bb.finish().map_err(|e| match e {
        yack_springrep::Error::InvalidResult(issues) => Error::Rejected(issues),
        other => Error::Rejected(vec![other.to_string()]),
    })?;
    Ok((b, map))
}
