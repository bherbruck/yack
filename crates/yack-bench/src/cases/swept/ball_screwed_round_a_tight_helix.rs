// yack: brep-bench case swept/ball-screwed-round-a-tight-helix
//! A ball screwed round a helix tighter than itself.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let (o, z) = (Point3::from([0.0; 3]), Vec3::from([0.0, 0.0, 1.0]));
    let screw = Motion::screw(o, z, rad(0.0), deg(720.0), Lead::per_turn(3.0));
    sphere(&Frame::at([1.5, 0.0, 0.0]), 2.0)?.swept(&screw)
}
