// yack: brep-bench case swept/tumbling-cube
//! A cube tumbled through space.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    // one roll t from -75° to 75°: t/4 about the upright line through (2, 0, 0), then −t about the line through
    // (0, 0.5, 0.5) along x, both at once
    let (p, z) = (Point3::new(2.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0));
    let (b, x) = (Point3::new(0.0, 0.5, 0.5), Vec3::new(1.0, 0.0, 0.0));
    let cube = block(&Frame::at([2.4, -0.6, 1.4]), [1.2, 1.2, 1.2])?;
    let roll = Motion::turn(p, z, deg(-18.75), deg(18.75));
    let tumble = Motion::turn(b, x, deg(75.0), deg(-75.0));
    cube.swept(&Motion::together(vec![roll, tumble]))
}
