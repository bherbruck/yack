// yack: the protocol's view axes and small numeric helpers the facade routes through.
use yack_base::Result;

use crate::{Dir3, Point2, Point3, Vec3};

/// The view-plane axes looking along `d`, as brep-conformance defines them: `u = d × z` made unit
/// (`d × x` when `d` runs along z), across, and `v = −d × u`, up.
pub fn view_axes(d: impl Into<Vec3>) -> Result<(Dir3, Dir3)> {
    let d = Dir3::new(d).map_err(|e| e.in_op("project"))?.vec();
    let across = d.cross(Vec3::new(0.0, 0.0, 1.0));
    let u = if across.length() <= yack_tol::TANGENT { Dir3::new(d.cross(Vec3::new(1.0, 0.0, 0.0)))? } else { Dir3::new(across)? };
    let v = Dir3::new((-d).cross(u.vec()))?;
    Ok((u, v))
}

/// `p` drawn in the view plane with axes `u`, `v`: `(p·u, p·v)`.
pub fn view_point(p: Point3, u: Dir3, v: Dir3) -> Point2 {
    Point2::new(p.to_vec().dot(u.vec()), p.to_vec().dot(v.vec()))
}

/// The parameter halfway between `a` and `b`.
pub fn mean(a: f64, b: f64) -> f64 {
    0.5 * (a + b)
}

/// `p` scaled by `s` about the origin of its plane.
pub fn scale2(p: Point2, s: f64) -> Point2 {
    Point2::new(p.x * s, p.y * s)
}

/// A length scaled by `s`.
pub fn scale_length(l: f64, s: f64) -> f64 {
    l * s
}
