// yack: brep-bench case loft/twisted-thin-blade
//! A thin blade twisted through a full turn.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let axis = Spline3::bezier(&[[0.0, 0.0, 0.0], [0.0, 0.0, 60.0]])?;
    sweep_path(&Frame::WORLD, &Profile::rectangle([-20.0, -0.2], [20.0, 0.2])?, &axis, deg(360.0))
}
