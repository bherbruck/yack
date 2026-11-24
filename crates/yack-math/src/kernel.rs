// yack: Points, vectors, frames, rigid transforms and boxes. A point and a vector share [`Vec3`]; the
// Adapted from springrep@a03e8a1 src/math.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: `within` moved in from tol.rs (M2), being the one tolerance compare that takes a point;
// moved to yack_math::kernel (M3 W2', D26); paths and visibility only.
//! Points, vectors, frames, rigid transforms and boxes. A point and a vector share [`Vec3`]; the
//! name at each use says which it is.
//!
//! springrep's own maths, beside yack-math's checked types. Temporary (D26): each crate's M3
//! hardening moves onto those types, and this module goes when the last user has.

pub mod bvh;

use std::ops::{Add, AddAssign, Div, Index, Mul, Neg, Sub, SubAssign};

use yack_base::kernel::{Result, invalid};
use yack_tol as tol;

/// `d <= t`, read with a few ulps of the coordinates' size at `at` to spare, so that what lies at
/// exactly a tolerance counts as within it however its coordinates round: a copy moved by exactly
/// [`tol::LINEAR`] lies 1e-7 mm plus a few ulps away.
#[inline]
pub fn within(d: f64, t: f64, at: Point3) -> bool {
    d <= t + 16.0 * f64::EPSILON * (1.0 + at.x.abs().max(at.y.abs()).max(at.z.abs()))
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

pub type Point3 = Vec3;

impl Vec3 {
    pub const ZERO: Vec3 = Vec3::new(0.0, 0.0, 0.0);
    pub const X: Vec3 = Vec3::new(1.0, 0.0, 0.0);
    pub const Y: Vec3 = Vec3::new(0.0, 1.0, 0.0);
    pub const Z: Vec3 = Vec3::new(0.0, 0.0, 1.0);

    #[inline]
    pub const fn new(x: f64, y: f64, z: f64) -> Vec3 {
        Vec3 { x, y, z }
    }

    #[inline]
    pub fn from_array(a: [f64; 3]) -> Vec3 {
        Vec3::new(a[0], a[1], a[2])
    }

    #[inline]
    pub fn to_array(self) -> [f64; 3] {
        [self.x, self.y, self.z]
    }

    #[inline]
    pub fn dot(self, o: Vec3) -> f64 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }

    #[inline]
    pub fn cross(self, o: Vec3) -> Vec3 {
        Vec3::new(
            self.y * o.z - self.z * o.y,
            self.z * o.x - self.x * o.z,
            self.x * o.y - self.y * o.x,
        )
    }

    #[inline]
    pub fn norm2(self) -> f64 {
        self.dot(self)
    }

    #[inline]
    pub fn norm(self) -> f64 {
        self.norm2().sqrt()
    }

    /// The unit vector along `self`, or `None` when it is too short to have a direction.
    #[inline]
    pub fn unit(self) -> Option<Vec3> {
        let n = self.norm();
        if n <= 1e-300 || !n.is_finite() {
            None
        } else {
            Some(self / n)
        }
    }

    /// The unit vector along `self`, or the zero vector.
    #[inline]
    pub fn unit_or_zero(self) -> Vec3 {
        self.unit().unwrap_or(Vec3::ZERO)
    }

    #[inline]
    pub fn lerp(self, o: Vec3, t: f64) -> Vec3 {
        self + (o - self) * t
    }

    #[inline]
    pub fn dist(self, o: Vec3) -> f64 {
        (self - o).norm()
    }

    /// The angle between two nonzero vectors, in [0, π].
    #[inline]
    pub fn angle(self, o: Vec3) -> f64 {
        self.cross(o).norm().atan2(self.dot(o))
    }

    /// Some unit vector perpendicular to `self`.
    #[inline]
    pub fn any_perpendicular(self) -> Vec3 {
        let a = if self.x.abs() < 0.6 { Vec3::X } else { Vec3::Y };
        self.cross(a).unit_or_zero()
    }

    #[inline]
    pub fn max_abs(self) -> f64 {
        self.x.abs().max(self.y.abs()).max(self.z.abs())
    }

    #[inline]
    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }
}

impl Index<usize> for Vec3 {
    type Output = f64;
    #[inline]
    fn index(&self, i: usize) -> &f64 {
        match i {
            0 => &self.x,
            1 => &self.y,
            _ => &self.z,
        }
    }
}

impl Add for Vec3 {
    type Output = Vec3;
    #[inline]
    fn add(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}

impl AddAssign for Vec3 {
    #[inline]
    fn add_assign(&mut self, o: Vec3) {
        *self = *self + o;
    }
}

impl Sub for Vec3 {
    type Output = Vec3;
    #[inline]
    fn sub(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}

impl SubAssign for Vec3 {
    #[inline]
    fn sub_assign(&mut self, o: Vec3) {
        *self = *self - o;
    }
}

impl Mul<f64> for Vec3 {
    type Output = Vec3;
    #[inline]
    fn mul(self, s: f64) -> Vec3 {
        Vec3::new(self.x * s, self.y * s, self.z * s)
    }
}

impl Mul<Vec3> for f64 {
    type Output = Vec3;
    #[inline]
    fn mul(self, v: Vec3) -> Vec3 {
        v * self
    }
}

impl Div<f64> for Vec3 {
    type Output = Vec3;
    #[inline]
    fn div(self, s: f64) -> Vec3 {
        Vec3::new(self.x / s, self.y / s, self.z / s)
    }
}

impl Neg for Vec3 {
    type Output = Vec3;
    #[inline]
    fn neg(self) -> Vec3 {
        Vec3::new(-self.x, -self.y, -self.z)
    }
}

/// A point or vector in a surface's parameter plane, or in a sketch.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec2 {
    pub x: f64,
    pub y: f64,
}

impl Vec2 {
    pub const ZERO: Vec2 = Vec2::new(0.0, 0.0);

    #[inline]
    pub const fn new(x: f64, y: f64) -> Vec2 {
        Vec2 { x, y }
    }

    #[inline]
    pub fn dot(self, o: Vec2) -> f64 {
        self.x * o.x + self.y * o.y
    }

    /// The z component of the 3-D cross product.
    #[inline]
    pub fn cross(self, o: Vec2) -> f64 {
        self.x * o.y - self.y * o.x
    }

    #[inline]
    pub fn norm(self) -> f64 {
        self.dot(self).sqrt()
    }

    #[inline]
    pub fn dist(self, o: Vec2) -> f64 {
        (self - o).norm()
    }

    #[inline]
    pub fn lerp(self, o: Vec2, t: f64) -> Vec2 {
        self + (o - self) * t
    }

    #[inline]
    pub fn perp(self) -> Vec2 {
        Vec2::new(-self.y, self.x)
    }
}

impl Add for Vec2 {
    type Output = Vec2;
    #[inline]
    fn add(self, o: Vec2) -> Vec2 {
        Vec2::new(self.x + o.x, self.y + o.y)
    }
}

impl Sub for Vec2 {
    type Output = Vec2;
    #[inline]
    fn sub(self, o: Vec2) -> Vec2 {
        Vec2::new(self.x - o.x, self.y - o.y)
    }
}

impl Mul<f64> for Vec2 {
    type Output = Vec2;
    #[inline]
    fn mul(self, s: f64) -> Vec2 {
        Vec2::new(self.x * s, self.y * s)
    }
}

impl Div<f64> for Vec2 {
    type Output = Vec2;
    #[inline]
    fn div(self, s: f64) -> Vec2 {
        Vec2::new(self.x / s, self.y / s)
    }
}

impl Neg for Vec2 {
    type Output = Vec2;
    #[inline]
    fn neg(self) -> Vec2 {
        Vec2::new(-self.x, -self.y)
    }
}

/// A right-handed orthonormal frame. Surfaces and conics are defined in one: `z` is a plane's
/// normal and a cylinder's or circle's axis, and angles are measured from `x` towards `y`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Frame {
    pub origin: Point3,
    pub x: Vec3,
    pub y: Vec3,
    pub z: Vec3,
}

impl Frame {
    pub const WORLD: Frame = Frame {
        origin: Vec3::ZERO,
        x: Vec3::X,
        y: Vec3::Y,
        z: Vec3::Z,
    };

    /// The frame at `origin` with axis `z` and `x` as near `x_hint` as perpendicularity allows.
    pub fn new(origin: Point3, z: Vec3, x_hint: Vec3) -> Result<Frame> {
        let Some(z) = z.unit() else {
            return invalid("frame normal is zero");
        };
        let x = x_hint - z * x_hint.dot(z);
        let Some(x) = x.unit().filter(|_| x.norm() > tol::ANGULAR * x_hint.norm()) else {
            return invalid("frame x direction is parallel to its normal");
        };
        Ok(Frame {
            origin,
            x,
            y: z.cross(x),
            z,
        })
    }

    /// A frame with axis `z` and some `x`.
    pub fn from_axis(origin: Point3, z: Vec3) -> Result<Frame> {
        Frame::new(origin, z, z.any_perpendicular())
    }

    #[inline]
    pub fn to_local(&self, p: Point3) -> Vec3 {
        let d = p - self.origin;
        Vec3::new(d.dot(self.x), d.dot(self.y), d.dot(self.z))
    }

    #[inline]
    pub fn to_world(&self, l: Vec3) -> Point3 {
        self.origin + self.x * l.x + self.y * l.y + self.z * l.z
    }

    #[inline]
    pub fn dir_to_world(&self, l: Vec3) -> Vec3 {
        self.x * l.x + self.y * l.y + self.z * l.z
    }

    #[inline]
    pub fn dir_to_local(&self, d: Vec3) -> Vec3 {
        Vec3::new(d.dot(self.x), d.dot(self.y), d.dot(self.z))
    }

    /// The point at `(u, v)` in the frame's xy plane.
    #[inline]
    pub fn at(&self, u: f64, v: f64) -> Point3 {
        self.origin + self.x * u + self.y * v
    }

    /// The radial direction at angle `a` in the xy plane.
    #[inline]
    pub fn radial(&self, a: f64) -> Vec3 {
        self.x * a.cos() + self.y * a.sin()
    }

    pub fn transformed(&self, t: &Transform) -> Frame {
        Frame {
            origin: t.point(self.origin),
            x: t.vector(self.x),
            y: t.vector(self.y),
            z: t.vector(self.z),
        }
    }

    pub fn translated(&self, d: Vec3) -> Frame {
        Frame {
            origin: self.origin + d,
            ..*self
        }
    }

    /// The motion taking world coordinates to this frame's: local `x` lands on `self.x`.
    pub fn to_transform(&self) -> Transform {
        let (x, y, z) = (self.x, self.y, self.z);
        Transform {
            r: [[x.x, y.x, z.x], [x.y, y.y, z.y], [x.z, y.z, z.z]],
            t: self.origin,
        }
    }
}

/// A rigid motion: rotation `r` (rows) then translation `t`. `r` must be a rotation to rounding:
/// numbers from a file or another program go through [`Transform::rigid`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Transform {
    pub r: [[f64; 3]; 3],
    pub t: Vec3,
}

/// How far `m·mᵀ` may be from the identity for `m` to be taken as an orthogonal matrix written
/// with few digits: six digits' worth.
const ORTHOGONAL: f64 = 1e-6;

fn det3(m: &[[f64; 3]; 3]) -> f64 {
    m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
}

/// The orthogonal matrix nearest `m`, a rotation or a reflection as `m` is, or `None` where `m`
/// is not one to within [`ORTHOGONAL`]. A matrix written to nine digits is 1e-9 from orthogonal,
/// and a frame moved by it has axes that far from unit and from square, more than the kernel's
/// angular tolerance.
pub fn nearest_orthogonal(m: &[[f64; 3]; 3]) -> Option<[[f64; 3]; 3]> {
    for i in 0..3 {
        for j in 0..3 {
            let dot: f64 = (0..3).map(|k| m[i][k] * m[j][k]).sum();
            let want = if i == j { 1.0 } else { 0.0 };
            let off = (dot - want).abs();
            if off.is_nan() || off > ORTHOGONAL {
                return None;
            }
        }
    }
    // The orthogonal factor of the polar decomposition, by Newton's iteration q ← ½(q + q⁻ᵀ),
    // which converges quadratically: from 1e-6 off, two steps reach rounding.
    let mut q = *m;
    for _ in 0..4 {
        let d = det3(&q);
        let mut next = [[0.0; 3]; 3];
        for (i, row) in next.iter_mut().enumerate() {
            for (j, cell) in row.iter_mut().enumerate() {
                let (a, b) = ((i + 1) % 3, (i + 2) % 3);
                let (c, e) = ((j + 1) % 3, (j + 2) % 3);
                let cofactor = q[a][c] * q[b][e] - q[a][e] * q[b][c];
                *cell = 0.5 * (q[i][j] + cofactor / d);
            }
        }
        q = next;
    }
    Some(q)
}

impl Transform {
    /// The rigid motion by the rotation nearest `r` (rows) then `t`, for numbers that come from
    /// outside. A matrix written to a few digits is a rotation only to those digits; one further
    /// than six digits from a rotation, or a reflection, is an error.
    pub fn rigid(r: [[f64; 3]; 3], t: Vec3) -> Result<Transform> {
        match nearest_orthogonal(&r) {
            Some(q) if det3(&q) > 0.0 => Ok(Transform { r: q, t }),
            Some(_) => invalid("a rigid motion's matrix must be a rotation, not a reflection"),
            None => invalid("a rigid motion's matrix must be a rotation"),
        }
    }

    pub const IDENTITY: Transform = Transform {
        r: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        t: Vec3::ZERO,
    };

    pub fn translation(d: Vec3) -> Transform {
        Transform {
            t: d,
            ..Transform::IDENTITY
        }
    }

    /// The right-handed rotation by `angle` about the line through `origin` along `axis`.
    pub fn rotation(origin: Point3, axis: Vec3, angle: f64) -> Transform {
        let k = axis.unit_or_zero();
        let (s, c) = angle.sin_cos();
        let v = 1.0 - c;
        let r = [
            [
                c + k.x * k.x * v,
                k.x * k.y * v - k.z * s,
                k.x * k.z * v + k.y * s,
            ],
            [
                k.y * k.x * v + k.z * s,
                c + k.y * k.y * v,
                k.y * k.z * v - k.x * s,
            ],
            [
                k.z * k.x * v - k.y * s,
                k.z * k.y * v + k.x * s,
                c + k.z * k.z * v,
            ],
        ];
        let rot = Transform { r, t: Vec3::ZERO };
        Transform {
            r,
            t: origin - rot.vector(origin),
        }
    }

    #[inline]
    pub fn vector(&self, v: Vec3) -> Vec3 {
        let r = &self.r;
        Vec3::new(
            r[0][0] * v.x + r[0][1] * v.y + r[0][2] * v.z,
            r[1][0] * v.x + r[1][1] * v.y + r[1][2] * v.z,
            r[2][0] * v.x + r[2][1] * v.y + r[2][2] * v.z,
        )
    }

    #[inline]
    pub fn point(&self, p: Point3) -> Point3 {
        self.vector(p) + self.t
    }

    /// `self` followed by `next`.
    pub fn then(&self, next: &Transform) -> Transform {
        let mut r = [[0.0; 3]; 3];
        for (i, row) in r.iter_mut().enumerate() {
            for (j, cell) in row.iter_mut().enumerate() {
                *cell = (0..3).map(|k| next.r[i][k] * self.r[k][j]).sum();
            }
        }
        Transform {
            r,
            t: next.point(self.t),
        }
    }

    pub fn inverse(&self) -> Transform {
        let mut r = [[0.0; 3]; 3];
        for (i, row) in r.iter_mut().enumerate() {
            for (j, cell) in row.iter_mut().enumerate() {
                *cell = self.r[j][i];
            }
        }
        let inv = Transform { r, t: Vec3::ZERO };
        Transform {
            r,
            t: -inv.vector(self.t),
        }
    }
}

/// An axis-aligned box. An empty box has `min > max`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Aabb {
    pub min: Point3,
    pub max: Point3,
}

impl Default for Aabb {
    fn default() -> Aabb {
        Aabb::EMPTY
    }
}

impl Aabb {
    pub const EMPTY: Aabb = Aabb {
        min: Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY),
        max: Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY),
    };

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.min.x > self.max.x
    }

    #[inline]
    pub fn add(&mut self, p: Point3) {
        self.min = Vec3::new(
            self.min.x.min(p.x),
            self.min.y.min(p.y),
            self.min.z.min(p.z),
        );
        self.max = Vec3::new(
            self.max.x.max(p.x),
            self.max.y.max(p.y),
            self.max.z.max(p.z),
        );
    }

    #[inline]
    pub fn union(&mut self, o: &Aabb) {
        if !o.is_empty() {
            self.add(o.min);
            self.add(o.max);
        }
    }

    #[inline]
    pub fn diagonal(&self) -> f64 {
        if self.is_empty() {
            0.0
        } else {
            (self.max - self.min).norm()
        }
    }

    #[inline]
    pub fn center(&self) -> Point3 {
        (self.min + self.max) * 0.5
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rotation_is_right_handed_and_inverts() {
        let t = Transform::rotation(
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::Z,
            std::f64::consts::FRAC_PI_2,
        );
        let p = t.point(Vec3::new(2.0, 0.0, 5.0));
        assert!(p.dist(Vec3::new(1.0, 1.0, 5.0)) < 1e-15);
        let back = t.inverse().point(p);
        assert!(back.dist(Vec3::new(2.0, 0.0, 5.0)) < 1e-15);
        let both = t.then(&t);
        assert!(both.point(Vec3::new(2.0, 0.0, 0.0)).dist(Vec3::ZERO) < 1e-15);
    }

    #[test]
    fn frame_orthonormalises_its_hint() {
        let f = Frame::new(
            Vec3::ZERO,
            Vec3::new(0.0, -1.0, 0.0),
            Vec3::new(1.0, 0.3, 0.0),
        )
        .unwrap();
        assert!(f.x.dist(Vec3::X) < 1e-15);
        assert!(f.y.dist(Vec3::Z) < 1e-15);
        assert!(Frame::new(Vec3::ZERO, Vec3::Z, Vec3::Z).is_err());
    }
}
