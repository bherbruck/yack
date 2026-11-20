//! A cube with every edge chamfered, its corners mitered.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let cube = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    let mids = cube.edges().iter().map(|&e| Ok(cube.edge(e)?.midpoint())).collect::<Result<Vec<_>>>()?;
    mids.into_iter().try_fold(cube, |c, p| c.chamfer(&[c.edge_at(p)?], 2.0))
}
