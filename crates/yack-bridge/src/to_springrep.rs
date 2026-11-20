// yack: cadrs `Body` → springrep `Brep` (MAPPING.md §1–§4, the C→S column).
use std::collections::BTreeSet;

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
    if rational {
        let w = n.cps.iter().map(|h| h.w).collect();
        Ok(SCurve::Nurbs(Box::new(SNurbsCurve::new(n.degree, n.knots.clone(), ctrl, Some(w)).map_err(reject)?)))
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

/// The springrep `Brep` of `body`, giving face `i` the names `names[i]` (none past its end).
pub fn to_springrep(body: &Body, names: &[BTreeSet<FaceName>]) -> Result<Brep> {
    if !body.wires().is_empty() || !body.points().is_empty() {
        return Err(Error::Unsupported("a body with wires or acorn points".into()));
    }
    if body.shells().iter().any(|s| !s.closed) {
        return Err(Error::Unsupported("a body with an open shell (a sheet)".into()));
    }
    let mut bb = Builder::new();
    let verts: Vec<VertexId> = body.verts().iter().map(|vx| bb.vertex(v(vx.p), vx.tol)).collect();
    let vert = |i: usize| verts.get(i).copied().ok_or_else(|| Error::Unsupported(format!("vertex {i} out of range")));
    let mut edges: Vec<EdgeId> = Vec::with_capacity(body.edges().len());
    for e in body.edges() {
        let range = (e.t[0], e.t[1]);
        edges.push(if e.degenerate {
            bb.degenerate_edge(vert(e.v[0])?, range)
        } else {
            bb.edge(curve(&e.curve)?, range, vert(e.v[0])?, vert(e.v[1])?, e.tol)
        });
    }
    let mut faces = Vec::with_capacity(body.faces().len());
    for (i, f) in body.faces().iter().enumerate() {
        let id = bb.face(surface(&f.surf)?, !f.sense, names.get(i).cloned().unwrap_or_default().into_iter().collect());
        for lp in &f.loops {
            let mut specs = Vec::with_capacity(lp.len());
            for co in lp {
                let edge = edges.get(co.edge).copied().ok_or_else(|| Error::Unsupported(format!("edge {} out of range", co.edge)))?;
                let t = body.edges().get(co.edge).map_or([0.0, 0.0], |e| e.t);
                specs.push(CoedgeSpec { edge, reversed: !co.fwd, pcurve: pcurve(&co.pc, t, co.fwd) });
            }
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
    bb.finish().map_err(|e| match e {
        yack_springrep::Error::InvalidResult(issues) => Error::Rejected(issues),
        other => Error::Rejected(vec![other.to_string()]),
    })
}
