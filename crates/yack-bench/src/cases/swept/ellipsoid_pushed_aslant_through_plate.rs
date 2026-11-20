// yack: brep-bench case swept/ellipsoid-pushed-aslant-through-plate
//! An ellipsoid pushed aslant wholly through a plate.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let stretch = Affine::from_rows([[3.0, 0.0, 0.0, -2.0], [0.0, 2.0, 0.0, -1.5], [0.0, 0.0, 1.5, -4.0]])?;
    let egg = sphere(&Frame::WORLD, 1.0)?.transform_affine(&stretch)?;
    let plate = block(&Frame::at([-15.0, -15.0, 0.0]), [30.0, 30.0, 4.0])?;
    plate.cut_swept(&egg, &Motion::slide([4.0, 3.0, 12.0]))
}
