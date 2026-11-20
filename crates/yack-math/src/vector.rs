// yack: points, vectors and unit directions (PLAN §4.1).
use core::ops::{Add, Mul, Neg, Sub};

use yack_base::{Error, Result};

/// A point in the plane (mm).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Point2 {
    /// x coordinate.
    pub x: f64,
    /// y coordinate.
    pub y: f64,
}

/// A point in space (mm).
///
/// `[f64; 3]` converts into it, so every API taking `impl Into<Point3>` accepts `[1.0, 2.0, 3.0]`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Point3 {
    /// x coordinate.
    pub x: f64,
    /// y coordinate.
    pub y: f64,
    /// z coordinate.
    pub z: f64,
}

/// A displacement in space (mm), of any length.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec3 {
    /// x component.
    pub x: f64,
    /// y component.
    pub y: f64,
    /// z component.
    pub z: f64,
}

/// A unit direction in space. Built only by [`Dir3::new`], which normalises and refuses zero.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Dir3(Vec3);

impl Point2 {
    /// The point `(x, y)`.
    pub const fn new(x: f64, y: f64) -> Point2 {
        Point2 { x, y }
    }
    /// Whether both coordinates are finite.
    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite()
    }
    /// The distance to `other`.
    pub fn distance(self, other: Point2) -> f64 {
        (self.x - other.x).hypot(self.y - other.y)
    }
    /// The coordinates as an array.
    pub const fn to_array(self) -> [f64; 2] {
        [self.x, self.y]
    }
}

impl From<[f64; 2]> for Point2 {
    fn from(a: [f64; 2]) -> Point2 {
        Point2 { x: a[0], y: a[1] }
    }
}

impl Point3 {
    /// The origin.
    pub const ORIGIN: Point3 = Point3 { x: 0.0, y: 0.0, z: 0.0 };
    /// The point `(x, y, z)`.
    pub const fn new(x: f64, y: f64, z: f64) -> Point3 {
        Point3 { x, y, z }
    }
    /// Whether all coordinates are finite.
    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }
    /// The distance to `other`.
    pub fn distance(self, other: Point3) -> f64 {
        (self - other).length()
    }
    /// The displacement from the origin to this point.
    pub const fn to_vec(self) -> Vec3 {
        Vec3 { x: self.x, y: self.y, z: self.z }
    }
    /// The coordinates as an array.
    pub const fn to_array(self) -> [f64; 3] {
        [self.x, self.y, self.z]
    }
}

impl From<[f64; 3]> for Point3 {
    fn from(a: [f64; 3]) -> Point3 {
        Point3 { x: a[0], y: a[1], z: a[2] }
    }
}

impl Vec3 {
    /// The zero vector.
    pub const ZERO: Vec3 = Vec3 { x: 0.0, y: 0.0, z: 0.0 };
    /// The vector `(x, y, z)`.
    pub const fn new(x: f64, y: f64, z: f64) -> Vec3 {
        Vec3 { x, y, z }
    }
    /// Whether all components are finite.
    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }
    /// The dot product.
    pub fn dot(self, o: Vec3) -> f64 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }
    /// The cross product.
    pub fn cross(self, o: Vec3) -> Vec3 {
        Vec3 {
            x: self.y * o.z - self.z * o.y,
            y: self.z * o.x - self.x * o.z,
            z: self.x * o.y - self.y * o.x,
        }
    }
    /// The Euclidean length.
    pub fn length(self) -> f64 {
        self.dot(self).sqrt()
    }
    /// The components as an array.
    pub const fn to_array(self) -> [f64; 3] {
        [self.x, self.y, self.z]
    }
}

impl From<[f64; 3]> for Vec3 {
    fn from(a: [f64; 3]) -> Vec3 {
        Vec3 { x: a[0], y: a[1], z: a[2] }
    }
}

impl From<Dir3> for Vec3 {
    fn from(d: Dir3) -> Vec3 {
        d.0
    }
}

impl Dir3 {
    /// +x.
    pub const X: Dir3 = Dir3(Vec3 { x: 1.0, y: 0.0, z: 0.0 });
    /// +y.
    pub const Y: Dir3 = Dir3(Vec3 { x: 0.0, y: 1.0, z: 0.0 });
    /// +z.
    pub const Z: Dir3 = Dir3(Vec3 { x: 0.0, y: 0.0, z: 1.0 });

    /// `v` made unit. Refuses a zero or non-finite vector.
    ///
    /// ```
    /// use yack_math::Dir3;
    /// assert_eq!(Dir3::new([0.0, 0.0, 2.0]).unwrap(), Dir3::Z);
    /// assert!(Dir3::new([0.0, 0.0, 0.0]).is_err());
    /// ```
    pub fn new(v: impl Into<Vec3>) -> Result<Dir3> {
        let v = v.into();
        let l = v.length();
        if !v.is_finite() || !l.is_finite() || l <= yack_tol::ANGULAR {
            return Err(Error::invalid(
                "Dir3::new",
                "direction",
                format!("({}, {}, {}) has no direction", v.x, v.y, v.z),
            ));
        }
        Ok(Dir3(v * (1.0 / l)))
    }
    /// The unit vector.
    pub const fn vec(self) -> Vec3 {
        self.0
    }
}

impl Add<Vec3> for Vec3 {
    type Output = Vec3;
    fn add(self, o: Vec3) -> Vec3 {
        Vec3 { x: self.x + o.x, y: self.y + o.y, z: self.z + o.z }
    }
}

impl Sub<Vec3> for Vec3 {
    type Output = Vec3;
    fn sub(self, o: Vec3) -> Vec3 {
        Vec3 { x: self.x - o.x, y: self.y - o.y, z: self.z - o.z }
    }
}

impl Neg for Vec3 {
    type Output = Vec3;
    fn neg(self) -> Vec3 {
        Vec3 { x: -self.x, y: -self.y, z: -self.z }
    }
}

impl Mul<f64> for Vec3 {
    type Output = Vec3;
    fn mul(self, s: f64) -> Vec3 {
        Vec3 { x: self.x * s, y: self.y * s, z: self.z * s }
    }
}

impl Add<Vec3> for Point3 {
    type Output = Point3;
    fn add(self, v: Vec3) -> Point3 {
        Point3 { x: self.x + v.x, y: self.y + v.y, z: self.z + v.z }
    }
}

impl Sub<Vec3> for Point3 {
    type Output = Point3;
    fn sub(self, v: Vec3) -> Point3 {
        Point3 { x: self.x - v.x, y: self.y - v.y, z: self.z - v.z }
    }
}

impl Sub<Point3> for Point3 {
    type Output = Vec3;
    fn sub(self, p: Point3) -> Vec3 {
        Vec3 { x: self.x - p.x, y: self.y - p.y, z: self.z - p.z }
    }
}
