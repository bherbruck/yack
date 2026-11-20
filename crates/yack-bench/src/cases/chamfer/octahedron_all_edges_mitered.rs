// yack: brep-bench case chamfer/octahedron-all-edges-mitered
//! An octahedron with every edge chamfered, its corners mitered.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let diamond = Profile::polygon(&[[10.0, 0.0], [0.0, 10.0], [-10.0, 0.0], [0.0, -10.0]])?;
    let oct = (0..8).try_fold(extrude(&Frame::at([0.0, 0.0, -10.0]), &diamond, 20.0)?, |s, i| {
        let n = [1, 2, 4].map(|b| if i & b == 0 { 1.0 } else { -1.0 });
        s.split(&Plane::new(n.map(|c| c * 10.0 / 3.0), n)?).map(|cut| cut.back)
    })?;
    let mids = oct.edges().iter().map(|&e| Ok(oct.edge(e)?.midpoint())).collect::<Result<Vec<_>>>()?;
    mids.into_iter().try_fold(oct, |s, p| s.chamfer(&[s.edge_at(p)?], 6f64.sqrt()))
}
