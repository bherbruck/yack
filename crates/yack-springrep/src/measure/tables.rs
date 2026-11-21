// yack: each projected coedge's surface parameters along its edge, seeding the measures' projections.

use crate::math::Vec2;
use crate::topo::{Brep, CoedgeId, FaceId, PCurve, coedge_params};

/// How many even steps along a coedge its table of parameters takes at the least, and per span of
/// its curve (a swept spline's edges have hundreds: between 64 entries a blended hint missed the
/// foot and every projection searched the whole surface, bench swept/ring-carried-along-a-spline),
/// and at most.
const TABLE: usize = 64;
const TABLE_PER_SPAN: usize = 4;
const TABLE_MAX: usize = 4096;

/// Each projected coedge's surface parameters at [`TABLE`] + 1 even steps along its edge in its
/// own direction, loop by loop; empty for a straight pcurve. They seed every projection the face's
/// integrals make, and its memo keeps them for each later integral.
pub(super) fn pcurve_tables(b: &Brep, f: FaceId) -> &[Vec<Vec<Vec2>>] {
    b.memo(f).pcurves.get_or_init(|| {
        b.face(f)
            .loops
            .iter()
            .map(|&lp| {
                b.lp(lp)
                    .coedges
                    .iter()
                    .map(|&co| pcurve_table(b, co))
                    .collect()
            })
            .collect()
    })
}

/// One coedge's entry of [`pcurve_tables`].
pub(super) fn pcurve_table(b: &Brep, co: CoedgeId) -> Vec<Vec2> {
    let coedge = b.coedge(co);
    if !matches!(coedge.pcurve, PCurve::Projected { .. }) || b.edge(coedge.edge).curve.is_none() {
        return Vec::new();
    }
    let edge = b.edge(coedge.edge);
    let spans =
        edge.curve.as_ref().map_or(0, |c| c.breakpoints(edge.range.0, edge.range.1).len() + 1);
    let n = (TABLE_PER_SPAN * spans).clamp(TABLE, TABLE_MAX);
    b.coedge_uvs(co, &coedge_params(b, co, n))
}

