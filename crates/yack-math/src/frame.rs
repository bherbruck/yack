// yack: right-handed orthonormal frames, checked and never repaired (PLAN §4.2 rule 5, D10).
use yack_base::{Error, Result};

use crate::{Aabb, Dir3, Point3, Rigid, Vec3};

/// A right-handed orthonormal frame: an origin and unit axes x, y, z with `x × y = z`.
///
/// Every yack constructor places its shape in a frame. [`Frame::new`] refuses a frame that is not
/// unit, not orthogonal or left-handed, naming the axis; it never repairs one.
///
/// ```
/// use yack_math::Frame;
/// let f = Frame::new([0.0, 0.0, 5.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]).unwrap();
/// assert_eq!(f.origin().z, 5.0);
/// // left-handed: refused
/// assert!(Frame::new([0.0; 3], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]).is_err());
/// ```
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Frame {
    origin: Point3,
    x: Dir3,
    y: Dir3,
    z: Dir3,
}

impl Frame {
    /// The world frame: origin at zero, axes x, y, z.
    pub const WORLD: Frame = Frame { origin: Point3::ORIGIN, x: Dir3::X, y: Dir3::Y, z: Dir3::Z };

    /// The frame with the given origin and axes. Refuses a non-finite value, an axis that is not
    /// unit, two axes that are not perpendicular, or `x × y ≠ z`.
    pub fn new(
        origin: impl Into<Point3>,
        x: impl Into<Vec3>,
        y: impl Into<Vec3>,
        z: impl Into<Vec3>,
    ) -> Result<Frame> {
        let op = "Frame::new";
        let tol = yack_tol::ANGULAR;
        let origin = origin.into();
        if !origin.is_finite() {
            return Err(Error::invalid(op, "origin", "not finite"));
        }
        let axes = [("x", x.into()), ("y", y.into()), ("z", z.into())];
        for (name, v) in axes {
            let l = v.length();
            if !v.is_finite() || (l - 1.0).abs() > tol {
                return Err(Error::invalid(op, name, format!("axis ({}, {}, {}) has length {l}, not 1", v.x, v.y, v.z)));
            }
        }
        let [(_, x), (_, y), (_, z)] = axes;
        for (name, a, b) in [("y", x, y), ("z", y, z), ("z", x, z)] {
            let d = a.dot(b);
            if d.abs() > tol {
                return Err(Error::invalid(op, name, format!("axes are not perpendicular (dot {d})")));
            }
        }
        if x.cross(y).dot(z) <= 0.0 {
            return Err(Error::invalid(op, "z", "the frame is left-handed: x × y points along -z"));
        }
        let unit = |v: Vec3| Dir3::new(v).map_err(|e| e.in_op(op));
        Ok(Frame { origin, x: unit(x)?, y: unit(y)?, z: unit(z)? })
    }

    /// The world axes moved to `origin`.
    pub fn at(origin: impl Into<Point3>) -> Frame {
        Frame { origin: origin.into(), ..Frame::WORLD }
    }

    /// The frame at `origin` whose z axis runs along `z` (made unit). Its x axis is the world x
    /// made square to `z`, or the world y when `z` runs along the world x. Refuses a zero `z`.
    ///
    /// ```
    /// use yack_math::{Dir3, Frame};
    /// let f = Frame::along([1.0, 2.0, 3.0], [0.0, 0.0, -5.0]).unwrap();
    /// assert_eq!(f.z(), Dir3::new([0.0, 0.0, -1.0]).unwrap());
    /// assert_eq!(f.x(), Dir3::X);
    /// ```
    pub fn along(origin: impl Into<Point3>, z: impl Into<Vec3>) -> Result<Frame> {
        let op = "Frame::along";
        let origin = origin.into();
        if !origin.is_finite() {
            return Err(Error::invalid(op, "origin", "not finite"));
        }
        let z = Dir3::new(z).map_err(|e| e.in_op(op))?;
        let zv = z.vec();
        let hint = if zv.cross(Vec3::new(1.0, 0.0, 0.0)).length() <= yack_tol::TANGENT {
            Vec3::new(0.0, 1.0, 0.0)
        } else {
            Vec3::new(1.0, 0.0, 0.0)
        };
        let x = Dir3::new(hint - zv * hint.dot(zv)).map_err(|e| e.in_op(op))?;
        let y = Dir3::new(zv.cross(x.vec())).map_err(|e| e.in_op(op))?;
        Ok(Frame { origin, x, y, z })
    }

    /// This frame turned by `by` about its own z axis (counter-clockwise looking down z).
    pub fn turned(&self, by: crate::Angle) -> Frame {
        let (s, c) = by.radians().sin_cos();
        let (x, y) = (self.x.vec(), self.y.vec());
        let x2 = Dir3::new(x * c + y * s).unwrap_or(self.x);
        let y2 = Dir3::new(y * c - x * s).unwrap_or(self.y);
        Frame { x: x2, y: y2, ..*self }
    }

    /// This frame carried by the rigid motion `m`.
    pub fn moved(&self, m: &Rigid) -> Frame {
        // A rigid map keeps unit, orthogonal, right-handed axes; renormalising only removes
        // rounding, and an axis cannot vanish under a rotation.
        let axis = |d: Dir3| Dir3::new(m.vec(d.vec())).unwrap_or(d);
        Frame { origin: m.point(self.origin), x: axis(self.x), y: axis(self.y), z: axis(self.z) }
    }

    /// The origin.
    pub const fn origin(&self) -> Point3 {
        self.origin
    }
    /// The x axis.
    pub const fn x(&self) -> Dir3 {
        self.x
    }
    /// The y axis.
    pub const fn y(&self) -> Dir3 {
        self.y
    }
    /// The z axis.
    pub const fn z(&self) -> Dir3 {
        self.z
    }

    /// The world point at local coordinates `local`.
    pub fn point(&self, local: [f64; 3]) -> Point3 {
        let [a, b, c] = local;
        self.origin + self.x.vec() * a + self.y.vec() * b + self.z.vec() * c
    }

    /// The world box around the local box `[lo, hi]`.
    pub fn bound(&self, lo: [f64; 3], hi: [f64; 3]) -> Aabb {
        let mut b = Aabb::point(self.point(lo));
        for i in 0..8u8 {
            let pick = |bit: u8, l: f64, h: f64| if i & bit == 0 { l } else { h };
            b = b.with(self.point([pick(1, lo[0], hi[0]), pick(2, lo[1], hi[1]), pick(4, lo[2], hi[2])]));
        }
        b
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn skew_names_the_axis() {
        let e = Frame::new([0.0; 3], [1.0, 0.0, 0.0], [0.6, 0.8, 0.0], [0.0, 0.0, 1.0]).unwrap_err();
        assert_eq!(e.arg, Some("y"));
        let e = Frame::new([0.0; 3], [2.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]).unwrap_err();
        assert_eq!(e.arg, Some("x"));
    }

    #[test]
    fn along_and_turned_stay_orthonormal() {
        let f = Frame::along([0.0; 3], [1.0, 1.0, 1.0]).unwrap().turned(crate::deg(30.0));
        assert!(Frame::new(f.origin(), f.x(), f.y(), f.z()).is_ok());
        let g = Frame::along([0.0; 3], [2.0, 0.0, 0.0]).unwrap();
        assert_eq!(g.x(), Dir3::Y);
        let q = Frame::WORLD.turned(crate::deg(90.0));
        assert!((q.x().vec() - Vec3::new(0.0, 1.0, 0.0)).length() < yack_tol::LINEAR);
    }

    #[test]
    fn world_is_valid() {
        let w = Frame::WORLD;
        assert_eq!(Frame::new(w.origin(), w.x(), w.y(), w.z()).unwrap(), w);
    }
}
