// yack: the classify self-check on a body carried into springrep: both engines must place a fixed set of
// points alike, or the body is refused (a misclassified body would make springrep's next op wrong).
use yack_cadrs::PointClass;
use yack_cadrs::native::math::P2;
use yack_cadrs::native::{Body, P3, V3, classify};
use yack_springrep::{
    math::Vec3,
    rays::{Rays, Side},
    topo::Brep,
};

use crate::{Error, Result};

/// Grid points per axis over the body's box.
const GRID: usize = 4;

/// How far off a face, as a fraction of the box's diagonal, its two probe points lie.
const OFF_FACE: f64 = 1e-3; // tol: a probe's step off a face, well past both engines' tolerances

/// How far a face's probe point is drawn from its loop's first coedge toward the loop's middle.
const INTO_FACE: f64 = 0.1; // tol: a fraction of the way into the face, not a length

/// The points both engines classify: a grid over the box grown by a tenth, nudged off symmetry
/// planes, and two points just off each face (outside and inside along its normal).
fn probes(body: &Body) -> Vec<P3> {
    let bb = body.bbox();
    let (lo, hi) = (bb.min, bb.max);
    let d = hi - lo;
    let diag = d.norm();
    let grow = d * 0.1;
    let (lo, hi) = (lo - grow, hi + grow);
    let at = |n: usize, a: f64, b: f64| a + (b - a) * (n as f64 + 0.37) / GRID as f64;
    let mut out = Vec::new();
    for i in 0..GRID {
        for j in 0..GRID {
            for k in 0..GRID {
                out.push(P3::new(at(i, lo.x, hi.x), at(j, lo.y, hi.y), at(k, lo.z, hi.z)));
            }
        }
    }
    for f in body.faces() {
        let Some(lp) = f.loops.first() else { continue };
        let uv_of = |co: &yack_cadrs::native::Coedge| {
            body.edges().get(co.edge).map(|e| co.pc.point(0.5 * (e.t[0] + e.t[1])))
        };
        let pts: Vec<P3> = lp.iter().filter_map(uv_of).collect();
        let (Some(first), n) = (pts.first(), pts.len()) else { continue };
        let mean = pts.iter().fold(P2::new(0.0, 0.0), |m, p| P2::new(m.x + p.x / n as f64, m.y + p.y / n as f64));
        let uv = P2::new(first.x + (mean.x - first.x) * INTO_FACE, first.y + (mean.y - first.y) * INTO_FACE);
        let p = f.surf.eval(uv.x, uv.y);
        let nrm = f.surf.normal(uv.x, uv.y);
        let n = if f.sense { nrm } else { -nrm };
        let len = n.norm();
        if !(len > 0.0) || !p.coords.iter().all(|x| x.is_finite()) {
            continue;
        }
        let step: V3 = n * (OFF_FACE * diag / len);
        out.push(p + step);
        out.push(p - step);
    }
    out
}

/// Refuses `back` (springrep's form of `body`) where springrep and cadrs place a probe point apart.
/// A point either calls on the boundary is skipped: there the engines may round either way.
pub(crate) fn same_sides(body: &Body, back: &Brep) -> Result<()> {
    let rays = Rays::new(back).map_err(|e| Error::Rejected(vec![format!("no ray caster for the body: {e}")]))?;
    let polys: Vec<_> = body.faces().iter().map(|f| classify::face_polygons(body, f)).collect();
    for p in probes(body) {
        let c = classify::classify_point(body, &polys, &p, yack_springrep::tol::LINEAR);
        let s = rays.side(Vec3::new(p.x, p.y, p.z));
        let apart = matches!((c, s), (PointClass::Inside, Side::Outside) | (PointClass::Outside, Side::Inside));
        if apart {
            return Err(Error::Unsupported(format!(
                "springrep places ({}, {}, {}) {s:?}, cadrs {c:?}: the body did not cross intact",
                p.x, p.y, p.z
            )));
        }
    }
    Ok(())
}
