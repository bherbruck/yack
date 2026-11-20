// yack: brep-bench case swept/spheroid-wheel-screwed-through-rod
//! A rod threaded by a wheel whose section is an ellipse.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let (o, z) = (Point3::from([0.0; 3]), Vec3::from([0.0, 0.0, 1.0]));
    let h = (2.0 * std::f64::consts::PI * 4.75 / 30.0).atan();
    let (s, c) = (h.sin(), h.cos());
    // Semi-axes 6, 6, 1.5 about (10.5, 0, 0), as placed turned -90° about z and lowered 7.5.
    let a = Affine::from_rows([[0.0, 6.0 * s, 1.5 * c, 0.0], [-6.0, 0.0, 0.0, -10.5], [0.0, 6.0 * c, -1.5 * s, -7.5]])?;
    let wheel = sphere(&Frame::WORLD, 1.0)?.transform_affine(&a)?;
    let screw = Motion::Screw { axis_point: o, axis: z, angle: deg(420.0), rise_per_turn: 30.0 };
    cylinder(&Frame::WORLD, 5.0, 20.0)?.difference(&wheel.swept(&screw)?)
}
