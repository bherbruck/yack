// yack: a measure-only scan of a body's p-curves on B-spline faces, densely, against their edges.
//! Each coedge on a B-spline face sampled densely: how far its p-curve strays from its edge (what
//! `CheckLevel::Full` holds to the edge's tolerance), how far the edge itself lies from the face's
//! surface (is the solid off, or only the p-curve?), and how far the p-curve leaves the surface's
//! knot range (where a written B-spline surface doesn't exist). Measures only; refuses nothing.

use crate::native::surface::Surface;
use crate::{BodyId, DefaultKernel, Result};

/// Samples a coedge's parameter range is taken at.
const SAMPLES: u32 = 400;

/// What [`pcurve_scan`] found on a body's B-spline faces.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct PcurveScan {
    /// Coedges on B-spline faces looked at.
    pub coedges: usize,
    /// Of those, how many have a p-curve farther from its edge than the edge's tolerance.
    pub over_tol: usize,
    /// The largest p-curve-to-edge distance, |S(pc(t)) - C(t)|.
    pub pcurve_off: f64,
    /// Where it is: (face, edge, t).
    pub pcurve_at: (usize, usize, f64),
    /// The edge's tolerance there.
    pub tol_there: f64,
    /// The largest edge-to-surface distance (the nearer of the foot from the p-curve's point and
    /// the global one): how far the solid itself is off.
    pub edge_off: f64,
    /// The farthest a p-curve leaves its surface's knot range.
    pub outside: f64,
}

/// [`PcurveScan`] of `body`'s coedges on B-spline faces.
pub fn pcurve_scan(k: &DefaultKernel, body: BodyId) -> Result<PcurveScan> {
    let b = k.get(body)?;
    let mut s = PcurveScan::default();
    for (fi, f) in b.faces.iter().enumerate() {
        let Surface::Nurbs(n) = &f.surf else { continue };
        let ((u0, u1), (v0, v1)) = n.range();
        for c in f.loops.iter().flatten() {
            let Some(e) = b.edges.get(c.edge) else { continue };
            if e.degenerate {
                continue;
            }
            s.coedges += 1;
            let mut over = false;
            for i in 0..=SAMPLES {
                let t = e.t[0] + (e.t[1] - e.t[0]) * f64::from(i) / f64::from(SAMPLES);
                let (p, uv) = (e.point(t), c.pc.point(t));
                let d = (f.surf.eval(uv.x, uv.y) - p).norm();
                over |= d > e.tol;
                if d > s.pcurve_off {
                    (s.pcurve_off, s.pcurve_at, s.tol_there) = (d, (fi, c.edge, t), e.tol);
                }
                let near = |h: Option<(f64, f64)>| {
                    let (u, v) = f.surf.project(&p, h);
                    (f.surf.eval(u, v) - p).norm()
                };
                s.edge_off = s.edge_off.max(near(Some((uv.x, uv.y))).min(near(None)));
                s.outside = s.outside.max((u0 - uv.x).max(uv.x - u1).max(v0 - uv.y).max(uv.y - v1));
            }
            s.over_tol += usize::from(over);
        }
    }
    Ok(s)
}
