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
/// ```
/// use yack_math::*;
/// let p = turned_about(Point3::new(1.0, 0.0, 0.0), Point3::ORIGIN, Vec3::new(0.0, 0.0, 1.0), deg(90.0))?;
/// assert!(p.x.abs() < 1e-15 && (p.y - 1.0).abs() < 1e-15);
/// # Ok::<(), yack_base::Error>(())
/// ```
pub fn turned_about(at: Point3, axis_point: Point3, axis: Vec3, by: Angle) -> Result<Point3> {
    Ok(Rigid::rotation(axis_point, axis, by)?.point(at))
}
