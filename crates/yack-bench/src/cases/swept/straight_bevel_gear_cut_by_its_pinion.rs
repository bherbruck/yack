// yack: brep-bench case swept/straight-bevel-gear-cut-by-its-pinion
//! A straight bevel gear's tooth space cut by its pinion.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    // data: g along the line where the pitch cones touch, d square to it; the blank's quadrilateral s·g + u·d in (ρ, z)
    let (s5, tan20) = (5f64.sqrt(), deg(20.0).tan());
    let (g, d) = ([2.0 / s5, 0.0, 1.0 / s5], [-1.0 / s5, 0.0, 2.0 / s5]);
    let (ae, ai) = (15.0 * s5, 15.0 * s5 - 10.0);
    let at = |s: f64, u: f64| [s * g[0] + u * d[0], s * g[2] + u * d[2]];
    let quad = [at(ae, -2.0), at(ae, 6.0), at(ai, 6.0), at(ai, -2.0 * ai / ae)];
    let blank = revolve(&Frame::WORLD, &Profile::polygon(&quad)?, deg(360.0))?;
    // the tooth: the trapezoid |u| ≤ 2.5, |w| ≤ π/2 − u·tan 20° (at cone distance ae; u along d, w along −y), scaled
    // to the planes square to g at ae − 12 and ae + 2 and lofted between them: a frustum of the pyramid with apex O
    let w = |u: f64| std::f64::consts::FRAC_PI_2 - u * tan20;
    let trap = |k: f64| [[2.5, -w(2.5)], [2.5, w(2.5)], [-2.5, w(-2.5)], [-2.5, -w(-2.5)]].map(|[u, v]| [u * k, v * k]);
    let (inner, outer) = (ae - 12.0, ae + 2.0);
    let foot = Frame::new(g.map(|c| c * inner), d, [0.0, -1.0, 0.0], g)?;
    let tooth = loft(&foot, &Profile::polygon(&trap(inner / ae))?, outer / inner, outer - inner)?;
    // seen from the blank: turned by t about the x axis, then by t/2 about the z axis, t from −60° to 60°
    let pinion = Motion::turn(Point3::ORIGIN, Dir3::X, deg(-60.0), deg(60.0));
    let gear = Motion::turn(Point3::ORIGIN, Dir3::Z, deg(-30.0), deg(30.0));
    blank.cut_swept(&tooth, &Motion::together(vec![pinion, gear]))
}
