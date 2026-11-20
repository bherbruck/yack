// yack: brep-bench case chamfer/octahedron-all-edges
//! An octahedron with every edge chamfered, four chamfers meeting at each corner.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let diamond = Profile::polygon(&[[10.0, 0.0], [0.0, 10.0], [-10.0, 0.0], [0.0, -10.0]])?;
    let oct = (0..8).try_fold(extrude(&Frame::at([0.0, 0.0, -10.0]), &diamond, 20.0)?, |s, i| {
        let n = [1, 2, 4].map(|b| if i & b == 0 { 1.0 } else { -1.0 });
        s.split(&Plane::new(n.map(|c| c * 10.0 / 3.0), n)?).map(|cut| cut.back)
    })?;
    oct.chamfer(&oct.edges(), 6f64.sqrt())
}
