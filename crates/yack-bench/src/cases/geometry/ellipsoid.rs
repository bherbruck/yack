// yack: brep-bench case geometry/ellipsoid
//! An ellipsoid.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let stretch = Affine::from_rows([[6.0, 0.0, 0.0, 0.0], [0.0, 4.0, 0.0, 0.0], [0.0, 0.0, 3.0, 0.0]])?;
    sphere(&Frame::WORLD, 1.0)?.transform_affine(&stretch)
}
