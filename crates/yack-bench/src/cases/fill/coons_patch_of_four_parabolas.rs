// yack: brep-bench case fill/coons-patch-of-four-parabolas
//! A block capped by the Coons patch of four parabolas.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    // the cap is cadrs's fill of the four arcs; each side the flat face from its arc down to z = 0, then the floor
    let a = [
        [[0., 0., 10.], [5., 0., 14.], [10., 0., 10.]],
        [[10., 0., 10.], [10., 5., 6.], [10., 10., 10.]],
        [[10., 10., 10.], [5., 10., 6.], [0., 10., 10.]],
        [[0., 10., 10.], [0., 5., 14.], [0., 0., 10.]],
    ];
    let down = |p: [f64; 3]| [p[0], p[1], 0.0];
    let arcs = a.map(|c| Spline3::bezier(&c)).into_iter().collect::<Result<Vec<_>>>()?;
    let mut patches = vec![Patch::Fill { boundary: arcs.clone() }];
    for (arc, [p, _, q]) in arcs.into_iter().zip(a) {
        let rest = [[q, down(q)], [down(q), down(p)], [down(p), p]].map(|l| Spline3::bezier(&l));
        patches.push(Patch::PlanarFill { boundary: [Ok(arc)].into_iter().chain(rest).collect::<Result<Vec<_>>>()? });
    }
    let floor = a.map(|[p, _, q]| Spline3::bezier(&[down(p), down(q)])).into_iter().collect::<Result<Vec<_>>>()?;
    patches.push(Patch::PlanarFill { boundary: floor });
    sewn(&patches)
}
