// yack: points, vectors and unit directions (PLAN §4.1).
use core::ops::{Add, Mul, Neg, Sub};

use yack_base::{Error, Result};

/// A point in the plane (mm).
///
/// ```
/// use yack_math::*;
/// let p = Point2::new(1.0, 2.0);
/// assert!(p.is_finite() && p.to_array() == [1.0, 2.0]);
/// # Ok::<(), yack_base::Error>(())
/// ```
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
///
/// ```
/// use yack_math::*;
/// let p = Point3::new(1.0, 2.0, 3.0);
/// assert!(p.is_finite() && p.to_array() == [1.0, 2.0, 3.0] && p.to_vec() == Vec3::new(1.0, 2.0, 3.0));
/// # Ok::<(), yack_base::Error>(())
/// ```
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
///
/// ```
/// use yack_math::*;
/// let v = Vec3::new(1.0, 2.0, 3.0);
/// assert!(v.is_finite() && v.to_array() == [1.0, 2.0, 3.0] && v - v == Vec3::ZERO);
/// # Ok::<(), yack_base::Error>(())
/// ```
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
///
/// ```
/// use yack_math::*;
/// let d = Dir3::new([0.0, 0.0, 2.0])?; // made unit
/// assert_eq!(d.vec(), Vec3::new(0.0, 0.0, 1.0));
/// # Ok::<(), yack_base::Error>(())
/// ```
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Dir3(Vec3);

impl Dir3 {
    /// The direction at `latitude` above the xy plane and `longitude` round the z axis from x:
    /// `(cos lat · cos lon, cos lat · sin lon, sin lat)`.
    ///
    /// ```
    /// use yack_math::*;
    /// let d = Dir3::spherical(deg(90.0), deg(0.0));
    /// assert!((d.vec().z - 1.0).abs() < 1e-15);
    /// # Ok::<(), yack_base::Error>(())
    /// ```
    pub fn spherical(latitude: crate::Angle, longitude: crate::Angle) -> Dir3 {
        let (l, p) = (latitude.radians(), longitude.radians());
        Dir3(Vec3::new(l.cos() * p.cos(), l.cos() * p.sin(), l.sin()))
    }
}

impl Point2 {
    /// The point `(x, y)`.
    ///
    /// ```
    /// use yack_math::*;
    /// let p = Point2::new(3.0, 4.0);
    /// assert_eq!((p.x, p.y), (3.0, 4.0));
    /// # Ok::<(), yack_base::Error>(())
    /// ```
    pub const fn new(x: f64, y: f64) -> Point2 {
        Point2 { x, y }
    }
    /// The point `radius` from the origin at `angle` counter-clockwise from the x axis:
    /// `(radius·cos angle, radius·sin angle)`.
    ///
    /// ```
    /// use yack_math::*;
    /// let p = Point2::polar(2.0, deg(90.0));
    /// assert!(p.x.abs() < 1e-15 && (p.y - 2.0).abs() < 1e-15);
    /// # Ok::<(), yack_base::Error>(())
    /// ```
    pub fn polar(radius: f64, angle: crate::Angle) -> Point2 {
        Point2 { x: radius * angle.cos(), y: radius * angle.sin() }
    }
    /// Whether both coordinates are finite.
    ///
    /// (Its example is on [`Point2`].)
    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite()
    }
    /// The distance to `other`.
    ///
    /// ```
    /// use yack_math::*;
    /// assert_eq!(Point2::new(0.0, 0.0).distance(Point2::new(3.0, 4.0)), 5.0);
    /// # Ok::<(), yack_base::Error>(())
    /// ```
    pub fn distance(self, other: Point2) -> f64 {
        (self.x - other.x).hypot(self.y - other.y)
    }
    /// The coordinates as an array.
    ///
    /// (Its example is on [`Point2`].)
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
    ///
    /// ```
    /// use yack_math::*;
    /// let p = Point3::new(1.0, 2.0, 3.0);
    /// assert_eq!(p - Point3::ORIGIN, Vec3::new(1.0, 2.0, 3.0));
    /// # Ok::<(), yack_base::Error>(())
    /// ```
    pub const fn new(x: f64, y: f64, z: f64) -> Point3 {
        Point3 { x, y, z }
    }
    /// Whether all coordinates are finite.
    ///
    /// (Its example is on [`Point3`].)
    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }
    /// The distance to `other`.
    ///
    /// ```
    /// use yack_math::*;
    /// assert_eq!(Point3::new(1.0, 2.0, 2.0).distance(Point3::ORIGIN), 3.0);
    /// # Ok::<(), yack_base::Error>(())
    /// ```
    pub fn distance(self, other: Point3) -> f64 {
        (self - other).length()
    }
    /// The displacement from the origin to this point.
    ///
    /// (Its example is on [`Point3`].)
    pub const fn to_vec(self) -> Vec3 {
        Vec3 { x: self.x, y: self.y, z: self.z }
    }
    /// The coordinates as an array.
    ///
    /// (Its example is on [`Point3`].)
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
    ///
    /// ```
    /// use yack_math::*;
    /// let v = Vec3::new(1.0, 2.0, 2.0);
    /// assert_eq!(v * 2.0, Vec3::new(2.0, 4.0, 4.0));
    /// # Ok::<(), yack_base::Error>(())
    /// ```
    pub const fn new(x: f64, y: f64, z: f64) -> Vec3 {
        Vec3 { x, y, z }
    }
    /// Whether all components are finite.
    ///
    /// (Its example is on [`Vec3`].)
    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }
    /// The dot product.
    ///
    /// ```
    /// use yack_math::*;
    /// assert_eq!(Vec3::new(1.0, 2.0, 3.0).dot(Vec3::new(4.0, 5.0, 6.0)), 32.0);
    /// # Ok::<(), yack_base::Error>(())
    /// ```
    pub fn dot(self, o: Vec3) -> f64 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }
    /// The cross product.
    ///
    /// ```
    /// use yack_math::*;
    /// assert_eq!(Vec3::new(1.0, 0.0, 0.0).cross(Vec3::new(0.0, 1.0, 0.0)), Vec3::new(0.0, 0.0, 1.0));
    /// # Ok::<(), yack_base::Error>(())
    /// ```
    pub fn cross(self, o: Vec3) -> Vec3 {
        Vec3 {
            x: self.y * o.z - self.z * o.y,
            y: self.z * o.x - self.x * o.z,
            z: self.x * o.y - self.y * o.x,
        }
    }
    /// The Euclidean length.
    ///
    /// ```
    /// use yack_math::*;
    /// assert_eq!(Vec3::new(1.0, 2.0, 2.0).length(), 3.0);
    /// # Ok::<(), yack_base::Error>(())
    /// ```
    pub fn length(self) -> f64 {
        self.dot(self).sqrt()
    }
    /// The components as an array.
    ///
    /// (Its example is on [`Vec3`].)
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
    /// `v` taken as already unit (the caller has checked it within tolerance), kept bit for bit.
    pub(crate) const fn unchecked(v: Vec3) -> Dir3 {
        Dir3(v)
    }
    /// The unit vector.
    ///
    /// (Its example is on [`Dir3`].)
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
