//! A regular tetrahedron grown by 1.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let cube = block(&Frame::at([-3.0, -3.0, -3.0]), [6.0, 6.0, 6.0])?;
    let inward = [[1.0, 1.0, 1.0], [1.0, -1.0, -1.0], [-1.0, 1.0, -1.0], [-1.0, -1.0, 1.0]];
    let tetra = inward.iter().try_fold(cube, |s, &n| s.split(&Plane::new(n.map(|x: f64| -x), n)?).map(|p| p.front))?;
    tetra.offset(1.0)
}
