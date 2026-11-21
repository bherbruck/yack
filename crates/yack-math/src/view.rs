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

/// The volume a closed, outward triangle mesh encloses (divergence theorem); triangles with an
/// index out of range are skipped.
pub fn mesh_volume(positions: &[Point3], triangles: &[[u32; 3]]) -> f64 {
    let at = |i: u32| positions.get(i as usize).map(|p| p.to_vec());
    triangles
        .iter()
        .filter_map(|&[a, b, c]| Some(at(a)?.dot(at(b)?.cross(at(c)?))))
        .sum::<f64>()
        / 6.0
}

/// The steps between consecutive points: `points[i + 1] − points[i]`.
pub fn steps(points: &[Point3]) -> Vec<Vec3> {
    points.windows(2).filter_map(|w| match w {
        [a, b] => Some(Vec3::new(b.x - a.x, b.y - a.y, b.z - a.z)),
        _ => None,
    }).collect()
}

/// A rise per radian as the rise per whole turn.
pub fn per_turn_of(per_radian: f64) -> f64 {
    per_radian * core::f64::consts::TAU
}

/// How far `got` is from `want`, relative to `want` (to 1 where `want` is smaller).
pub fn relative_gap(got: f64, want: f64) -> f64 {
    (got - want).abs() / want.abs().max(1.0)
}

/// An angle in radians as whole turns.
pub fn turns_of(radians: f64) -> f64 {
    radians / core::f64::consts::TAU
}

/// A rise per whole turn as the rise per radian.
pub fn per_radian_of(per_turn: f64) -> f64 {
    per_turn / core::f64::consts::TAU
}
