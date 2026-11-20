// yack: brep-bench case swept/square-on-a-planet-carrier
//! A square bar carried round on a planet gear.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    // one parameter t from 0° to 360°: the bar turned by −3t about its own centre, then carried by t about the z axis
    let bar = block(&Frame::at([4.0, -1.0, 0.0]), [2.0, 2.0, 2.0])?;
    let z: Vec3 = Vec3::new(0.0, 0.0, 1.0);
    let spin = Motion::turn(Point3::new(5.0, 0.0, 0.0), z, deg(0.0), deg(-1080.0));
    let carry = Motion::turn(Point3::ORIGIN, z, deg(0.0), deg(360.0));
    bar.swept(&Motion::together(vec![spin, carry]))
}
