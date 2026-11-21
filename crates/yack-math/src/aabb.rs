// yack: axis-aligned boxes.
use crate::Point3;

/// An axis-aligned box `[min, max]`, `min ≤ max` on every axis.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Aabb {
    /// The low corner.
    pub min: Point3,
    /// The high corner.
    pub max: Point3,
}

impl Aabb {
    /// The smallest box holding both corners, in any order.
    ///
    /// ```
    /// use yack_math::*;
    /// // from any two opposite corners
    /// let bb = Aabb::new([1.0, 5.0, 0.0], [3.0, 2.0, 1.0]);
    /// assert_eq!((bb.min, bb.max), (Point3::new(1.0, 2.0, 0.0), Point3::new(3.0, 5.0, 1.0)));
    /// # Ok::<(), yack_base::Error>(())
    /// ```
    pub fn new(a: impl Into<Point3>, b: impl Into<Point3>) -> Aabb {
        Aabb::point(a.into()).with(b.into())
    }
    /// The box holding one point.
    ///
    /// ```
    /// use yack_math::*;
    /// let bb = Aabb::point(Point3::new(1.0, 2.0, 3.0));
    /// assert_eq!(bb.min, bb.max);
    /// # Ok::<(), yack_base::Error>(())
    /// ```
    pub const fn point(p: Point3) -> Aabb {
        Aabb { min: p, max: p }
    }
    /// This box grown to hold `p`.
    ///
    /// ```
    /// use yack_math::*;
    /// let bb = Aabb::point(Point3::ORIGIN).with(Point3::new(1.0, 1.0, 1.0));
    /// assert_eq!(bb.max, Point3::new(1.0, 1.0, 1.0));
    /// # Ok::<(), yack_base::Error>(())
    /// ```
    pub fn with(self, p: Point3) -> Aabb {
        Aabb {
            min: Point3::new(self.min.x.min(p.x), self.min.y.min(p.y), self.min.z.min(p.z)),
            max: Point3::new(self.max.x.max(p.x), self.max.y.max(p.y), self.max.z.max(p.z)),
        }
    }
    /// The smallest box holding both boxes.
    ///
    /// ```
    /// use yack_math::*;
    /// let bb = Aabb::point(Point3::ORIGIN).union(Aabb::point(Point3::new(2.0, 2.0, 2.0)));
    /// assert!(bb.contains(Point3::new(1.0, 1.0, 1.0)));
    /// # Ok::<(), yack_base::Error>(())
    /// ```
    pub fn union(self, o: Aabb) -> Aabb {
        self.with(o.min).with(o.max)
    }
    /// Whether `p` lies in the box, boundary included.
    ///
    /// ```
    /// use yack_math::*;
    /// let bb = Aabb::new([0.0; 3], [1.0, 1.0, 1.0]);
    /// assert!(bb.contains(Point3::new(1.0, 0.5, 0.0)) && !bb.contains(Point3::new(2.0, 0.0, 0.0)));
    /// # Ok::<(), yack_base::Error>(())
    /// ```
    pub fn contains(&self, p: Point3) -> bool {
        (self.min.x..=self.max.x).contains(&p.x)
            && (self.min.y..=self.max.y).contains(&p.y)
            && (self.min.z..=self.max.z).contains(&p.z)
    }
}
