// yack: brep-bench case swept/hypoid-gear-cut-by-its-pinion
//! A hypoid gear's tooth space cut by its pinion.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    // data: the straight bevel case's blank and tooth (g, d as there), the tooth turned about z by φ
    let (s5, tan20) = (5f64.sqrt(), 20f64.to_radians().tan());
    let (g, d) = ([2.0 / s5, 0.0, 1.0 / s5], [-1.0 / s5, 0.0, 2.0 / s5]);
    let (ae, ai) = (15.0 * s5, 15.0 * s5 - 10.0);
    let at = |s: f64, u: f64| [s * g[0] + u * d[0], s * g[2] + u * d[2]];
    let quad = [at(ae, -2.0), at(ae, 6.0), at(ai, 6.0), at(ai, -2.0 * ai / ae)];
    let blank = revolve(&Frame::WORLD, &Profile::polygon(&quad)?, deg(360.0))?;
    let w = |u: f64| std::f64::consts::FRAC_PI_2 - u * tan20;
    let trap = |k: f64| [[2.5, -w(2.5)], [2.5, w(2.5)], [-2.5, w(-2.5)], [-2.5, -w(-2.5)]].map(|[u, v]| [u * k, v * k]);
    let (inner, outer, phi) = (ae - 12.0, ae + 2.0, (24.0 / (75.0 - 5.0 * s5)).asin());
    let turned = Rigid::rotation([0.0; 3], [0.0, 0.0, 1.0], rad(phi))?;
    let foot = Frame::new(g.map(|c| c * inner), d, [0.0, -1.0, 0.0], g)?.moved(&turned);
    let tooth = loft(&foot, &Profile::polygon(&trap(inner / ae))?, outer / inner, outer - inner)?;
    // seen from the blank: turned by t about the pinion's axis (through (0, 12, 0) along x), then by 7t/15 about z,
    // t from −70° to 70°
    let (p, x, z): (Point3, Vec3, Vec3) = ([0.0, 12.0, 0.0].into(), Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0));
    let pinion = Motion::turn(p, x, deg(-70.0), deg(70.0));
    let gear = Motion::turn(Point3::ORIGIN, z, deg(-98.0 / 3.0), deg(98.0 / 3.0));
    blank.cut_swept(&tooth, &Motion::together(vec![pinion, gear]))
}
