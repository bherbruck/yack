// yack: brep-bench case geometry/ellipsoid-half
//! An ellipsoid halved.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let stretch = Affine::from_rows([[6.0, 0.0, 0.0, 0.0], [0.0, 4.0, 0.0, 0.0], [0.0, 0.0, 3.0, 0.0]])?;
    let ellipsoid = sphere(&Frame::WORLD, 1.0)?.transform_affine(&stretch)?;
    Ok(ellipsoid.split(&Plane::new([0.0, 0.0, 0.0], [0.0, 0.0, 1.0])?)?.front)
}
