// yack: the position arithmetic of a toolpath: where a slide or a turn takes the tool.
use yack_base::Result;

use crate::{Angle, Point3, Rigid, Vec3};

/// The slide from `at` to `to`, or `None` when they are the same point (no move).
///
/// ```
/// use yack_math::*;
/// assert_eq!(slide_to(Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 2.0, 0.0)), Some(Vec3::new(0.0, 2.0, 0.0)));
/// assert_eq!(slide_to(Point3::ORIGIN, Point3::ORIGIN), None);
/// ```
pub fn slide_to(at: Point3, to: Point3) -> Option<Vec3> {
    (to != at).then(|| Vec3::new(to.x - at.x, to.y - at.y, to.z - at.z))
}

/// Where `at` goes when turned by `by` about the line through `axis_point` along `axis`.
///
/// It is worked out as a hand-written path would: the foot `q + h` on the axis, plus `cos · u + sin · w`,
/// where `u` is `at`'s offset across the axis and `w` is `u` turned a quarter. So a turn about a
/// coordinate axis gives `cx + r·cos`, `cy + r·sin` to the last bit and keeps the coordinate along
/// the axis as it is. (A whole turn is not snapped: `sin 2π` is not 0, as in hand-written code.)
/// The errors are [`Rigid::rotation`]'s.
///
/// ```
/// use yack_math::*;
/// let p = turned_about(Point3::new(1.0, 0.0, 0.0), Point3::ORIGIN, Vec3::new(0.0, 0.0, 1.0), deg(90.0))?;
/// assert!(p.x.abs() < 1e-15 && (p.y - 1.0).abs() < 1e-15);
/// // bit for bit what a hand-written arc gives
/// let a = deg(37.0);
/// let q = turned_about(Point3::new(30.0, 5.0, -2.0), Point3::new(20.0, 5.0, 7.0), Vec3::new(0.0, 0.0, 1.0), a)?;
/// assert_eq!(q, Point3::new(20.0 + 10.0 * a.cos(), 5.0 + 10.0 * a.sin(), -2.0));
/// let t = deg(-360.0);
/// let e = turned_about(Point3::new(80.0, 55.0, -4.0), Point3::new(60.0, 55.0, -4.0), Vec3::new(0.0, 0.0, 1.0), t)?;
/// assert_eq!(e, Point3::new(60.0 + 20.0 * t.cos(), 55.0 + 20.0 * t.sin(), -4.0));
/// # Ok::<(), yack_base::Error>(())
/// ```
pub fn turned_about(at: Point3, axis_point: Point3, axis: Vec3, by: Angle) -> Result<Point3> {
    // The same checks, and so the same errors, as the rotation the motion makes.
    Rigid::rotation(axis_point, axis, by)?;
    let k = crate::Dir3::new(axis)?.vec();
    let d = at - axis_point;
    let along = d.dot(k);
    let h = k * along;
    let u = Vec3::new(d.x - h.x, d.y - h.y, d.z - h.z);
    let w = k.cross(u);
    let (c, s) = (by.cos(), by.sin());
    // Where `at` has no offset across the axis in a coordinate, that coordinate stays `at`'s own.
    let coord = |a: f64, q: f64, h: f64, u: f64, w: f64| if u == 0.0 && w == 0.0 { a } else { q + h + c * u + s * w };
    Ok(Point3::new(
        coord(at.x, axis_point.x, h.x, u.x, w.x),
        coord(at.y, axis_point.y, h.y, u.y, w.y),
        coord(at.z, axis_point.z, h.z, u.z, w.z),
    ))
}

/// The radius at which a bull-nose cutter's flat end meets its corner round: `radius − corner`.
///
/// ```
/// use yack_math::*;
/// assert_eq!(flat_end_radius(4.0, 1.5), 2.5);
/// ```
pub fn flat_end_radius(radius: f64, corner: f64) -> f64 {
    radius - corner
}

/// How far a drill's point cone rises from its tip to the drill's full `radius`, for an included
/// `point` angle: `radius / tan(point / 2)`, and exactly `radius` for a 90° point.
///
/// ```
/// use yack_math::*;
/// assert_eq!(drill_point_height(2.5, deg(90.0)), 2.5);
/// assert!((drill_point_height(3.0, deg(120.0)) - 3.0 / 3f64.sqrt()).abs() < 1e-12);
/// ```
pub fn drill_point_height(radius: f64, point: Angle) -> f64 {
    if point == crate::deg(90.0) { radius } else { radius / (point.radians() * 0.5).tan() }
}
