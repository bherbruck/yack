// yack: brep-bench case swept/box-screwed-inside-a-rod
//! A sealed helical tunnel: a box screwed inside a rod.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let (o, z) = (Point3::from([0.0; 3]), Vec3::from([0.0, 0.0, 1.0]));
    let tunnel = block(&Frame::at([2.5, -0.5, 2.0]), [1.0, 1.0, 1.0])?;
    let tunnel = tunnel.swept(&Motion::screw(o, z, rad(0.0), deg(720.0), Lead::per_turn(4.0)))?;
    cylinder(&Frame::WORLD, 5.0, 20.0)?.difference(&tunnel)
}
