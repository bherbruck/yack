// yack: brep-bench case swept/rack-cut-tooth-space
//! A tooth space cut by a rack.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    // the tooth placed where the roll starts, slid 20·(−π/3) along z; its section drawn in (z, x), the prism along +y
    let (d, a, b) = (-20.0 * std::f64::consts::FRAC_PI_3, 0.842856, 3.026666);
    let tooth = Profile::polygon(&[[d - a, 18.0], [d + a, 18.0], [d + b, 24.0], [d - b, 24.0]])?;
    let side = Frame::new([0.0, -2.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])?;
    let rack = extrude(&side, &tooth, 10.0)?;
    // seen from the blank: slid 2π·20 per turn along the pitch line, then turned about the y axis, from −60° to 60°
    let turn = Motion::turn(Point3::ORIGIN, Dir3::Y, deg(-60.0), deg(60.0));
    let roll = Motion::together(vec![Motion::slide(Vec3::new(0.0, 0.0, -2.0 * d)), turn]);
    cylinder(&Frame::along([0.0; 3], [0.0, 1.0, 0.0])?, 22.0, 6.0)?.cut_swept(&rack, &roll)
}
