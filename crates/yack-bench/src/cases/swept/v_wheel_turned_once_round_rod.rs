// yack: brep-bench case swept/v-wheel-turned-once-round-rod
//! A rod grooved by a leaning V wheel turned once round it.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let (s, c, t) = (20f64.to_radians().sin(), 20f64.to_radians().cos(), 2.5 * 30f64.to_radians().tan());
    let hub = Frame::new([14.0, 0.0, 10.0], [-1.0, 0.0, 0.0], [0.0, -c, s], [0.0, s, c])?;
    let wheel = revolve(&hub, &Profile::polygon(&[[7.5, -t], [10.0, 0.0], [7.5, t]])?, deg(360.0))?;
    let turn = Motion::turn([0.0; 3], [0.0, 0.0, 1.0], rad(0.0), deg(360.0));
    cylinder(&Frame::WORLD, 5.0, 20.0)?.cut_swept(&wheel, &turn)
}
