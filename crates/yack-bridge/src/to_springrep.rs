// yack: cadrs `Body` → springrep `Brep` (MAPPING.md §1–§4, the C→S column).
use std::collections::{BTreeMap, BTreeSet};

use yack_cadrs::native::pcurve::closed_periods;
use yack_cadrs::native::{Body, Curve as CCurve, Frame as CFrame, NurbsCurve as CNurbsCurve, P3, Surface as CSurface, V3};
use yack_springrep::{
    facade::bridge::Builder,
    geom::{
        BSpline, Circle, Cone, Curve as SCurve, Cylinder, Ellipse, Extrusion, Line, NurbsCurve as SNurbsCurve,
        NurbsSurface as SNurbsSurface, Plane, Revolution, Sphere, Surface as SSurface, Torus,
    },
    math::{Frame as SFrame, Vec2, Vec3},
    topo::{Brep, CoedgeSpec, EdgeId, FaceName, LoopId, PCurve, VertexId},
};

use crate::{Error, Result};

/// How close, as a fraction of the period, a coedge must stay to a whole number of periods to lie
/// on the seam line.
const SEAM_SLACK: f64 = 1e-9; // tol: parameter round-off on a seam, relative to the period

/// springrep's `BSpline` stops at cubics; higher degrees go to its `NurbsCurve` (up to 15).
const BSPLINE_MAX_DEGREE: usize = 3;

/// The widest gap between an edge and its faces carried into springrep, as that edge's own
/// tolerance (springrep's checker then judges it): cadrs's own retry tolerance, which its blends on
/// spline faces reach (~2e-5 mm, bench fillet/bezier-dome-one-edge). Wider is refused.
const GAP_CAP: f64 = 1e-4; // tol: cadrs's widest boolean retry tolerance, mm (lead ruling)

/// Samples per coedge when measuring how far cadrs's pcurves, curves and vertices stray apart:
/// springrep's checker reads each coedge at 16 even steps, so the gap is measured there and halfway
/// between (a 9-point measure missed a 1.36e-7 mm point it then refused, bench
/// swept/ball-screwed-round-a-tight-helix).
const GAP_SAMPLES: usize = 33;

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
            // A straight pcurve is carried as it is (springrep's `Affine`), so its own points are
            // measured. Any other becomes springrep's `Projected`: springrep re-projects the exact
            // edge onto the surface and keeps nothing of cadrs's curve but a seed, so what counts is
            // how far the edge itself lies from the surface (bench swept/ball-screwed-round-a-tight-
            // helix: one cadrs pcurve 4.8e-4 off, its edge within 4.5e-7).
            let carried = matches!(pcurve(&co.pc, e.t, co.fwd, false), PCurve::Affine { .. });
            let worst = (0..GAP_SAMPLES)
                .map(|i| {
                    let t = e.t[0] + (e.t[1] - e.t[0]) * i as f64 / (GAP_SAMPLES - 1) as f64;
                    let uv = co.pc.point(t);
                    let q = e.curve.point(t);
                    if carried {
                        (f.surf.eval(uv.x, uv.y) - q).norm()
                    } else {
                        let (u, v) = f.surf.project(&q, Some((uv.x, uv.y)));
                        (f.surf.eval(u, v) - q).norm()
                    }
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
fn pcurve(pc: &CCurve, t: [f64; 2], fwd: bool, fitted: bool) -> PCurve {
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
            if let Some(curve) = fitted.then(|| fitted_of(pc)).flatten() {
                return PCurve::Fitted { curve: Box::new(curve) };
            }
            let p = pc.point(if fwd { t[0] } else { t[1] });
            PCurve::Projected { seed: Vec2::new(p.x, p.y) }
        }
    }
}

/// cadrs's pcurve `pc` as springrep's fitted one, its points `(u, v, 0)` by the edge's t: a
/// spline's homogeneous points `[w·u, w·v, 0, w]` divided out. `None` for any other curve.
fn fitted_of(pc: &CCurve) -> Option<SNurbsCurve> {
    let CCurve::Nurbs(n) = pc else { return None };
    let ctrl = n.cps.iter().map(|h| Vec3::new(h.x / h.w, h.y / h.w, 0.0)).collect();
    let weights = n.cps.iter().map(|h| h.w).collect();
    SNurbsCurve::new(n.degree, n.knots.clone(), ctrl, Some(weights)).ok()
}

/// A fitted pcurve moved by `d` in the parameter plane.
fn shifted(curve: &SNurbsCurve, d: Vec2) -> Result<PCurve> {
    let ctrl = curve.control_points().iter().map(|p| Vec3::new(p.x + d.x, p.y + d.y, 0.0)).collect();
    SNurbsCurve::new(curve.degree(), curve.knots().to_vec(), ctrl, curve.weights().map(<[f64]>::to_vec))
        .map(|c| PCurve::Fitted { curve: Box::new(c) })
        .map_err(|e| Error::Unsupported(format!("a fitted pcurve would not move: {e}")))
}

/// How far, relative, the springrep form's volume may be from cadrs's.
const MEASURE_AGREES: f64 = 1e-5; // tol: two engines' quadratures of one body (as CADRS_AGREE)

/// Refuses `back` where its volume differs from `body`'s (cadrs's own measure). Volume only:
/// cadrs's face-area quadrature is wrong on offset bodies (closing-offsets: 24 bodies refused that
/// were sound), while a lost face or a lost region always shows in the volume.
fn same_measure(body: &Body, back: &Brep) -> Result<()> {
    let cv = yack_cadrs::native::mass::body_volume(body);
    // To a thousandth of the agreement asked, not to the last digits (a swept spline body's exact
    // volume took most of a minute: bench swept/ring-carried-along-a-spline).
    let sv = yack_springrep::measure::volume_to(back, MEASURE_AGREES * 1e-3); // tol: a thousandth of the guard
    if (sv - cv).abs() / cv.abs().max(1.0) <= MEASURE_AGREES {
        Ok(())
    } else {
        Err(Error::Unsupported(format!("the body lost geometry crossing: springrep volume {sv}, cadrs's {cv}")))
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
fn normalise(lp: &mut [Uses], periods: [Option<f64>; 2]) -> Result<()> {
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
            u.pc = match &u.pc {
                PCurve::Affine { origin, dir } => PCurve::Affine { origin: add(*origin), dir: *dir },
                PCurve::Projected { seed } => PCurve::Projected { seed: add(*seed) },
                // `pcurve` makes none yet (M1.15's bridge step will).
                PCurve::Fitted { curve } => shifted(curve, d)?,
            };
        }
    }
    Ok(())
}

/// A cadrs face in a few words: its surface, its range and closed periods, and its loops' uv span.
fn describe_face(body: &Body, fi: usize, f: &yack_cadrs::native::Face) -> String {
    let mut span = ((f64::INFINITY, f64::NEG_INFINITY), (f64::INFINITY, f64::NEG_INFINITY));
    for co in f.loops.iter().flatten() {
        let Some(e) = body.edges().get(co.edge) else { continue };
        for i in 0..GAP_SAMPLES {
            let t = e.t[0] + (e.t[1] - e.t[0]) * i as f64 / (GAP_SAMPLES - 1) as f64;
            let w = co.pc.point(t);
            span = ((span.0.0.min(w.x), span.0.1.max(w.x)), (span.1.0.min(w.y), span.1.1.max(w.y)));
        }
    }
    let range = match &f.surf {
        CSurface::Nurbs(n) => format!(" range {:?} degrees ({}, {})", n.range(), n.du, n.dv),
        _ => String::new(),
    };
    format!("(cadrs face {fi}: {:?}{range} periods {:?} loops span {span:?})", f.surf.kind(), closed_periods(&f.surf))
}

/// `s` unrolled over the uv span the face's pcurves reach, where it is a closed B-spline surface
/// they run past; otherwise as it is.
fn cover_span(s: CSurface, body: &Body, f: &yack_cadrs::native::Face) -> Result<CSurface> {
    let CSurface::Nurbs(n) = &s else { return Ok(s) };
    let periods = closed_periods(&s);
    if periods.iter().all(Option::is_none) {
        return Ok(s);
    }
    let mut span = ((f64::INFINITY, f64::NEG_INFINITY), (f64::INFINITY, f64::NEG_INFINITY));
    for co in f.loops.iter().flatten() {
        let Some(e) = body.edges().get(co.edge) else { continue };
        for i in 0..GAP_SAMPLES {
            let t = e.t[0] + (e.t[1] - e.t[0]) * i as f64 / (GAP_SAMPLES - 1) as f64;
            let w = co.pc.point(t);
            span = ((span.0.0.min(w.x), span.0.1.max(w.x)), (span.1.0.min(w.y), span.1.1.max(w.y)));
        }
    }
    let ((u0, u1), (v0, v1)) = n.range();
    let slack = SEAM_SLACK * (u1 - u0).abs().max(v1 - v0).max(1.0);
    let inside_u = span.0.0 >= u0 - slack && span.0.1 <= u1 + slack;
    let inside_v = span.1.0 >= v0 - slack && span.1.1 <= v1 + slack;
    if inside_u && inside_v {
        return Ok(s);
    }
    let (mut ku, mut kv, mut cps) = (n.ku.clone(), n.kv.clone(), n.cps.clone());
    if !inside_u && periods[0].is_some() {
        (ku, cps) = crate::to_cadrs::unroll_u(n.du, ku, cps, span.0)?;
    }
    if !inside_v && periods[1].is_some() {
        let (k, rows) = crate::to_cadrs::unroll_u(n.dv, kv, crate::to_cadrs::transpose(cps), span.1)?;
        kv = k;
        cps = crate::to_cadrs::transpose(rows);
    }
    let m = yack_cadrs::native::NurbsSurface::new(n.du, n.dv, ku, kv, cps).map_err(|e| Error::Rejected(vec![e.to_string()]))?;
    Ok(CSurface::Nurbs(std::sync::Arc::new(m)))
}

/// The face's surface with its frame turned about its axis so that u = 0 is the lowest u its loops
/// reach (springrep's place for a seam), and by how much u moved (the pcurves move by it too).
fn seam_frame(s: &CSurface, body: &Body, f: &yack_cadrs::native::Face) -> (CSurface, f64) {
    // The seam: an edge the face uses twice, at one u; its lower u becomes 0.
    let seam = || {
        let mut seen = BTreeMap::new();
        for co in f.loops.iter().flatten() {
            *seen.entry(co.edge).or_insert(0usize) += 1;
        }
        f.loops
            .iter()
            .flatten()
            .filter(|co| seen.get(&co.edge).copied() == Some(2))
            .filter_map(|co| {
                let t = body.edges().get(co.edge)?.t;
                let (a, b) = (co.pc.point(t[0]).x, co.pc.point(t[1]).x);
                ((a - b).abs() <= SEAM_SLACK * std::f64::consts::TAU).then_some(a)
            })
            .fold(None, |m: Option<f64>, u| Some(m.map_or(u, |m| m.min(u))))
    };
    let low = || {
        if let Some(u) = seam() {
            return u;
        }
        f.loops
            .iter()
            .flatten()
            .flat_map(|co| {
                let t = body.edges().get(co.edge).map_or([0.0, 0.0], |e| e.t);
                [t[0], 0.5 * (t[0] + t[1]), t[1]].map(|x| co.pc.point(x).x)
            })
            .fold(f64::INFINITY, f64::min)
    };
    let turn = |fr: &CFrame| -> Option<(CFrame, f64)> {
        let l = low();
        if !l.is_finite() || l == 0.0 {
            return None;
        }
        let (sn, cs) = l.sin_cos();
        let x = fr.x * cs + fr.y * sn;
        Some((CFrame { o: fr.o, x, y: fr.z.cross(&x), z: fr.z }, l))
    };
    match s {
        CSurface::Cylinder { f: fr, r } => turn(fr).map_or((s.clone(), 0.0), |(f, l)| (CSurface::Cylinder { f, r: *r }, l)),
        CSurface::Cone { f: fr, r, a } => turn(fr).map_or((s.clone(), 0.0), |(f, l)| (CSurface::Cone { f, r: *r, a: *a }, l)),
        CSurface::Sphere { f: fr, r } => turn(fr).map_or((s.clone(), 0.0), |(f, l)| (CSurface::Sphere { f, r: *r }, l)),
        CSurface::Torus { f: fr, rmaj, rmin } => {
            turn(fr).map_or((s.clone(), 0.0), |(f, l)| (CSurface::Torus { f, rmaj: *rmaj, rmin: *rmin }, l))
        }
        other => (other.clone(), 0.0),
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
    // As it is first; only a body that will not cross so, and has spline faces, is crossed again
    // with cadrs's own pcurves on them (M1.15): where a spline face folds near an edge, springrep's
    // projected pcurve can land on the other sheet, and a fitted one cannot.
    match carried(body, names, false) {
        Ok(out) => Ok(out),
        Err(first) if FITTED_RETRY && body.faces().iter().any(|f| matches!(f.surf, CSurface::Nurbs(_))) => {
            carried(body, names, true).map_err(|_| first)
        }
        Err(first) => Err(first),
    }
}

/// Whether a body that will not cross as it is is crossed again with fitted pcurves. Off: on
/// ladder25 the bodies it carried meshed open (swept/tumbling-torus-full-turn, a wrong answer) and
/// ran springrep's booleans past the time limit (the crossed tumbling-torus cases), where cadrs's
/// form holds them correctly. On again only when (1) springrep's mesher samples a Fitted edge
/// the same way from both its faces, so a carried body meshes closed, and (2) a carried body's
/// boolean is checked for cost (or bounded) so it cannot run past cadrs's time on the same op.
const FITTED_RETRY: bool = false;

/// [`to_springrep_mapped`] in one way: with springrep's pcurves, or `fitted`, cadrs's own on
/// spline faces.
fn carried(body: &Body, names: &[BTreeSet<FaceName>], fitted: bool) -> Result<(Brep, Vec<Option<usize>>)> {
    if !body.wires().is_empty() || !body.points().is_empty() {
        return Err(Error::Unsupported("a body with wires or acorn points".into()));
    }
    if body.shells().iter().any(|s| !s.closed) {
        return Err(Error::Unsupported("a body with an open shell (a sheet)".into()));
    }
    let (bb, edges, loops) = assemble(body, names, &BTreeMap::new(), fitted)?;
    // A degenerate (pole) coedge runs from where the coedge before it leaves the pole to where the
    // one after it arrives; springrep works out those coedges' u at the pole its own way, so the
    // pole's pcurve is set from what springrep made of them, and the body built again.
    let poles = pole_pcurves(body, &bb, &loops);
    let (bb, edges) = if poles.is_empty() {
        (bb, edges)
    } else {
        let (bb, edges, _) = assemble(body, names, &poles, fitted)?;
        (bb, edges)
    };
    let map = edges.iter().map(|e| e.map(EdgeId::index)).collect();
    let b = bb.finish().map_err(|e| match e {
        yack_springrep::Error::InvalidResult(mut issues) => {
            // What the refused faces were in cadrs, to say what to fix.
            let named: BTreeSet<usize> = issues
                .iter()
                .filter_map(|m| m.strip_prefix("face ").and_then(|r| r.split(|c: char| !c.is_ascii_digit()).next()).and_then(|n| n.parse().ok()))
                .collect();
            for fi in named.into_iter().take(3) {
                if let Some(f) = body.faces().get(fi) {
                    issues.push(describe_face(body, fi, f));
                }
            }
            Error::Rejected(issues)
        }
        other => Error::Rejected(vec![other.to_string()]),
    })?;
    // Nothing may be lost crossing: the springrep form's volume is cadrs's own, to the
    // measuring agreement of two engines (bench swept/ring-lying-flat-turned-whole once came over at
    // 217.13 of cadrs's 761.10 with every other check passed).
    same_measure(body, &b)?;
    crate::selfcheck::same_sides(body, &b)?;
    Ok((b, map))
}

/// How many steps along a coedge springrep's checker samples it by (`check::SAMPLES`).
const END_SAMPLES: usize = 16;

/// Where each face's loops went: `loops[face][loop]`.
type Loops = Vec<Vec<LoopId>>;

/// The springrep pcurve each pole coedge should have, by (face, loop, coedge) of `body`.
fn pole_pcurves(body: &Body, bb: &Builder, loops: &Loops) -> BTreeMap<(usize, usize, usize), PCurve> {
    let b = bb.peek();
    let mut out = BTreeMap::new();
    for (fi, f) in body.faces().iter().enumerate() {
        for (li, lp) in f.loops.iter().enumerate() {
            let Some(&lid) = loops.get(fi).and_then(|l| l.get(li)) else { continue };
            let ids = &b.lp(lid).coedges;
            let n = ids.len();
            for (k, co) in lp.iter().enumerate() {
                if !body.edges().get(co.edge).is_some_and(|e| e.degenerate) || n < 3 {
                    continue;
                }
                let (Some(&prev), Some(&here), Some(&next)) = (ids.get((k + n - 1) % n), ids.get(k), ids.get((k + 1) % n)) else {
                    continue;
                };
                // Sampled along the whole coedge, as springrep's checker reads its ends: a periodic
                // projection follows its branch from the start.
                let end_of = |c: yack_springrep::topo::CoedgeId, start: bool| {
                    let (t0, t1) = b.edge(b.coedge(c).edge).range;
                    let (a, z) = if b.coedge(c).reversed { (t1, t0) } else { (t0, t1) };
                    let ts: Vec<f64> = (0..=END_SAMPLES).map(|i| if i == END_SAMPLES { z } else { a + (z - a) * i as f64 / END_SAMPLES as f64 }).collect();
                    let uvs = b.coedge_uvs(c, &ts);
                    if start { uvs.first().copied() } else { uvs.last().copied() }
                };
                let (Some(a), Some(z)) = (end_of(prev, false), end_of(next, true)) else { continue };
                let (t0, t1) = b.edge(b.coedge(here).edge).range;
                let (ts, te) = if b.coedge(here).reversed { (t1, t0) } else { (t0, t1) };
                if te == ts {
                    continue;
                }
                let dir = Vec2::new((z.x - a.x) / (te - ts), (z.y - a.y) / (te - ts));
                let origin = Vec2::new(a.x - dir.x * ts, a.y - dir.y * ts);
                out.insert((fi, li, k), PCurve::Affine { origin, dir });
            }
        }
    }
    out
}

/// Builds `body` into springrep, with `poles` in place of the pole coedges' own pcurves; unchecked.
fn assemble(
    body: &Body,
    names: &[BTreeSet<FaceName>],
    poles: &BTreeMap<(usize, usize, usize), PCurve>,
    fitted: bool,
) -> Result<(Builder, Vec<Option<EdgeId>>, Loops)> {
    let mut bb = Builder::new();
    let (edge_gap, vert_gap) = gaps(body);
    // Geometry that meets only to more than cadrs's loop gap is not carried: springrep's booleans
    // on it would answer for a solid that is not quite the one cadrs built.
    if let Some((i, g)) = edge_gap.iter().enumerate().find(|&(_, &g)| g > GAP_CAP) {
        return Err(Error::Unsupported(format!(
            "cadrs edge {i}: its curve and its faces meet only to {g:.1e} mm (over {GAP_CAP:e})"
        )));
    }
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
    let mut loops: Loops = Vec::with_capacity(body.faces().len());
    for (i, f) in body.faces().iter().enumerate() {
        // springrep keeps a revolved surface's seam at u = 0 of its frame (its ray caster relies on
        // it); cadrs's frame may sit anywhere about the axis. Turn the frame to the face's lowest u.
        let (surf, turn) = seam_frame(&f.surf, body, f);
        // A closed cadrs spline surface whose loops run past its knots (cadrs keeps them continuous
        // across the seam): unrolled over their span, so springrep's projections stay on it.
        let surf = cover_span(surf, body, f)?;
        let id = bb.face(surface(&surf)?, !f.sense, names.get(i).cloned().unwrap_or_default().into_iter().collect());
        let periods = closed_periods(&f.surf);
        let mut face_loops = Vec::with_capacity(f.loops.len());
        for (li, lp) in f.loops.iter().enumerate() {
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
                let at = |x: f64| {
                    let w = uv(x);
                    Vec2::new(w.x - turn, w.y)
                };
                let pc = match pcurve(&co.pc, t, co.fwd, fitted && matches!(f.surf, CSurface::Nurbs(_))) {
                    PCurve::Affine { origin, dir } => PCurve::Affine { origin: Vec2::new(origin.x - turn, origin.y), dir },
                    PCurve::Projected { seed } => PCurve::Projected { seed: Vec2::new(seed.x - turn, seed.y) },
                    PCurve::Fitted { curve } => shifted(&curve, Vec2::new(-turn, 0.0))?,
                };
                uses.push(Uses { pc, start: at(ts), mid: at(0.5 * (ts + te)), end: at(te) });
                ids.push((edge, !co.fwd));
            }
            normalise(&mut uses, periods)?;
            let specs = ids
                .into_iter()
                .zip(uses)
                .enumerate()
                .map(|(k, ((edge, reversed), u))| CoedgeSpec {
                    edge,
                    reversed,
                    pcurve: poles.get(&(i, li, k)).cloned().unwrap_or(u.pc),
                })
                .collect();
            face_loops.push(bb.add_loop(id, specs));
        }
        loops.push(face_loops);
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
    Ok((bb, edges, loops))
}
