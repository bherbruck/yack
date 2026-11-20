// yack: brep-bench case swept/plus-cut-wheel
//! A wheel cut by a plus always in mesh with it.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let arms = [[30.0, -4.0], [30.0, 4.0], [4.0, 4.0], [4.0, 30.0], [-4.0, 30.0], [-4.0, 4.0]];
    let rest = [[-30.0, 4.0], [-30.0, -4.0], [-4.0, -4.0], [-4.0, -30.0], [4.0, -30.0], [4.0, -4.0]];
    let tool = extrude(&Frame::at([0.0, 0.0, -10.0]), &Profile::polygon(&[arms, rest].concat())?, 10.0)?;
    // seen from the wheel: the tool turned by t about the z axis, then by 2t/3 about the wheel's axis, t to 540°
    let z: Vec3 = Vec3::new(0.0, 0.0, 1.0);
    let spin = Motion::turn(Point3::ORIGIN, z, deg(0.0), deg(540.0));
    let mesh = Motion::turn(Point3::new(-50.0, 0.0, 0.0), z, deg(0.0), deg(360.0));
    cylinder(&Frame::at([-50.0, 0.0, -8.0]), 40.0, 6.0)?.cut_swept(&tool, &Motion::together(vec![spin, mesh]))
}
