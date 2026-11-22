// yack: rigid, similarity and affine maps, validated from 3x4 rows (PLAN §4.3).
use yack_base::{Error, Result};

use crate::{Angle, Dir3, Point3, Vec3};

type Rows = [[f64; 4]; 3];

const IDENTITY: Rows = [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0]];

/// A rotation followed by a translation: rows `[R | t]`, `R` a proper rotation.
///
/// ```
/// use yack_math::*;
/// let m = Rigid::translation([1.0, 0.0, 0.0]);
/// assert_eq!(m.rows()[0], [1.0, 0.0, 0.0, 1.0]);
/// # Ok::<(), yack_base::Error>(())
/// ```
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Rigid(Rows);

/// A rigid map times a uniform scale, mirrors allowed: rows `[c·Q | t]`, `Q` orthogonal, `c > 0`.
///
/// ```
/// use yack_math::*;
/// let m = Similarity::from_rows([[2.0, 0.0, 0.0, 0.0], [0.0, 2.0, 0.0, 0.0], [0.0, 0.0, 2.0, 1.0]])?;
/// assert_eq!(m.rows()[2], [0.0, 0.0, 2.0, 1.0]);
/// # Ok::<(), yack_base::Error>(())
/// ```
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Similarity(Rows);

/// Any invertible affine map: rows `[A | t]`, `det A ≠ 0`.
///
/// ```
/// use yack_math::*;
/// let m = Affine::from_rows([[1.0, 1.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0]])?;
/// assert_eq!(m.rows()[0], [1.0, 1.0, 0.0, 0.0]);
/// # Ok::<(), yack_base::Error>(())
/// ```
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Affine(Rows);

fn column(m: &Rows, j: usize) -> Vec3 {
    let get = |i: usize| m.get(i).and_then(|r| r.get(j)).copied().unwrap_or(0.0);
    Vec3::new(get(0), get(1), get(2))
}

fn finite(op: &'static str, m: &Rows) -> Result<()> {
    if m.iter().flatten().all(|v| v.is_finite()) {
        Ok(())
    } else {
        Err(Error::invalid(op, "rows", "a matrix entry is not finite"))
    }
}

fn det(m: &Rows) -> f64 {
    column(m, 0).cross(column(m, 1)).dot(column(m, 2))
}

/// How far a rigid motion's linear part may be from orthonormal (in its columns' dot products)
/// and still be taken as the rotation nearest it: rows written to six digits or more.
const NEAR_ROTATION: f64 = 1e-6; // tol: a rotation written to six or more significant digits

/// Checks that the linear part's columns are mutually orthogonal and all of length `scale`, each
/// dot product within `tol` (relative to `scale²`).
fn orthogonal(op: &'static str, m: &Rows, scale: f64, tol: f64) -> Result<()> {
    let c = [column(m, 0), column(m, 1), column(m, 2)];
    let s2 = scale * scale;
    for (i, a) in c.iter().enumerate() {
        for (j, b) in c.iter().enumerate().skip(i) {
            let want = if i == j { s2 } else { 0.0 };
            let got = a.dot(*b);
            if (got - want).abs() > tol * s2 {
                let what = if i == j {
                    format!("column {i} of the linear part has squared length {got}, not {want}")
                } else {
                    format!("columns {i} and {j} of the linear part have dot {got}, not {want}")
                };
                return Err(Error::invalid(op, "rows", what));
            }
        }
    }
    Ok(())
}

/// `m` with its near-orthonormal linear part replaced by the rotation nearest it (the polar
/// factor, by Newton–Schulz steps `R ← R·(3I − RᵀR)/2`, each squaring the error), shift kept.
fn nearest_rotation(m: &Rows) -> Rows {
    let mut c = [column(m, 0), column(m, 1), column(m, 2)];
    for _ in 0..4 {
        let [a, b, d] = c;
        let step = |cj: Vec3| (cj * 3.0 - (a * a.dot(cj) + b * b.dot(cj) + d * d.dot(cj))) * 0.5;
        c = [step(a), step(b), step(d)];
    }
    let [a, b, d] = c;
    [[a.x, b.x, d.x, m[0][3]], [a.y, b.y, d.y, m[1][3]], [a.z, b.z, d.z, m[2][3]]]
}

fn apply_point(m: &Rows, p: Point3) -> Point3 {
    let row = |r: &[f64; 4]| r[0] * p.x + r[1] * p.y + r[2] * p.z + r[3];
    Point3::new(row(&m[0]), row(&m[1]), row(&m[2]))
}

fn apply_vec(m: &Rows, v: Vec3) -> Vec3 {
    let row = |r: &[f64; 4]| r[0] * v.x + r[1] * v.y + r[2] * v.z;
    Vec3::new(row(&m[0]), row(&m[1]), row(&m[2]))
}

impl Rigid {
    /// The identity.
    pub const IDENTITY: Rigid = Rigid(IDENTITY);

    /// From rows `[R | t]`. An `R` orthonormal to within 1e-6 (in its columns' dot products, as
    /// rows written to six digits or more are) is taken as the rotation nearest it; one farther
    /// off, a non-finite entry, or a mirror is refused.
    ///
    /// ```
    /// use yack_math::Rigid;
    /// assert!(Rigid::from_rows([[1.0, 0.0, 0.0, 5.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0]]).is_ok());
    /// // a quarter turn written to nine digits is that turn
    /// let r = Rigid::from_rows([[0.963144594, 0.268984183, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.268984183, -0.963144594, 0.0, 0.0]])?;
    /// let c = r.rows();
    /// assert!((c[0][0] * c[0][0] + c[1][0] * c[1][0] + c[2][0] * c[2][0] - 1.0).abs() < 1e-15);
    /// assert!(Rigid::from_rows([[2.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0]]).is_err());
    /// # Ok::<(), yack_base::Error>(())
    /// ```
    pub fn from_rows(m: [[f64; 4]; 3]) -> Result<Rigid> {
        let op = "Rigid::from_rows";
        finite(op, &m)?;
        orthogonal(op, &m, 1.0, NEAR_ROTATION)?;
        if det(&m) <= 0.0 {
            return Err(Error::invalid(op, "rows", "the linear part is a mirror, not a rotation"));
        }
        // rows orthonormal as given are kept bit for bit; only near ones are rounded to a rotation
        if orthogonal(op, &m, 1.0, yack_tol::ANGULAR).is_ok() {
            return Ok(Rigid(m));
        }
        Ok(Rigid(nearest_rotation(&m)))
    }

    /// A pure translation by `v`.
    ///
    /// ```
    /// use yack_math::*;
    /// let m = Rigid::translation([1.0, 2.0, 3.0]);
    /// assert_eq!(m.point(Point3::ORIGIN), Point3::new(1.0, 2.0, 3.0));
    /// # Ok::<(), yack_base::Error>(())
    /// ```
    pub fn translation(v: impl Into<Vec3>) -> Rigid {
        let v = v.into();
        Rigid([[1.0, 0.0, 0.0, v.x], [0.0, 1.0, 0.0, v.y], [0.0, 0.0, 1.0, v.z]])
    }

    /// A rotation by `by` (counter-clockwise looking down `axis`) about the line through
    /// `axis_point` along `axis`. Refuses a zero axis.
    ///
    /// ```
    /// use yack_math::*;
    /// // a quarter turn about z: x goes to y
    /// let m = Rigid::rotation(Point3::ORIGIN, Dir3::Z, deg(90.0))?;
    /// assert!((m.point(Point3::new(1.0, 0.0, 0.0)) - Point3::new(0.0, 1.0, 0.0)).length() < 1e-12);
    /// # Ok::<(), yack_base::Error>(())
    /// ```
    pub fn rotation(axis_point: impl Into<Point3>, axis: impl Into<Vec3>, by: Angle) -> Result<Rigid> {
        let op = "Rigid::rotation";
        let p = axis_point.into();
        if !p.is_finite() {
            return Err(Error::invalid(op, "axis_point", "not finite"));
        }
        if !by.is_finite() {
            return Err(Error::invalid(op, "by", "not finite"));
        }
        let k = Dir3::new(axis.into()).map_err(|e| e.in_op(op))?.vec();
        let (s, c) = by.radians().sin_cos();
        let t = 1.0 - c;
        let r = [
            [t * k.x * k.x + c, t * k.x * k.y - s * k.z, t * k.x * k.z + s * k.y],
            [t * k.x * k.y + s * k.z, t * k.y * k.y + c, t * k.y * k.z - s * k.x],
            [t * k.x * k.z - s * k.y, t * k.y * k.z + s * k.x, t * k.z * k.z + c],
        ];
        // p + R (q - p) = R q + (p - R p)
        let rp = Vec3::new(
            r[0][0] * p.x + r[0][1] * p.y + r[0][2] * p.z,
            r[1][0] * p.x + r[1][1] * p.y + r[1][2] * p.z,
            r[2][0] * p.x + r[2][1] * p.y + r[2][2] * p.z,
        );
        let tr = p.to_vec() - rp;
        Ok(Rigid([
            [r[0][0], r[0][1], r[0][2], tr.x],
            [r[1][0], r[1][1], r[1][2], tr.y],
            [r[2][0], r[2][1], r[2][2], tr.z],
        ]))
    }

    /// The rows `[R | t]`.
    ///
    /// (Its example is on [`Rigid`].)
    pub const fn rows(&self) -> [[f64; 4]; 3] {
        self.0
    }
    /// The image of a point.
    ///
    /// ```
    /// use yack_math::*;
    /// let m = Rigid::translation([0.0, 0.0, 5.0]);
    /// assert_eq!(m.point(Point3::new(1.0, 1.0, 1.0)), Point3::new(1.0, 1.0, 6.0));
    /// # Ok::<(), yack_base::Error>(())
    /// ```
    pub fn point(&self, p: Point3) -> Point3 {
        apply_point(&self.0, p)
    }
    /// The image of a vector (rotation only).
    ///
    /// ```
    /// use yack_math::*;
    /// // a vector turns but does not move
    /// let m = Rigid::translation([0.0, 0.0, 5.0]);
    /// assert_eq!(m.vec(Vec3::new(1.0, 0.0, 0.0)), Vec3::new(1.0, 0.0, 0.0));
    /// # Ok::<(), yack_base::Error>(())
    /// ```
    pub fn vec(&self, v: Vec3) -> Vec3 {
        apply_vec(&self.0, v)
    }
}

impl Similarity {
    /// The identity.
    pub const IDENTITY: Similarity = Similarity(IDENTITY);

    /// The reflection in the plane through `origin` square to `normal`.
    ///
    /// ```
    /// use yack_math::*;
    /// let m = Similarity::mirror([0.0, 0.0, 1.0], [0.0, 0.0, 1.0])?; // in the plane z = 1
    /// assert_eq!(m.point(Point3::new(2.0, 3.0, 4.0)), Point3::new(2.0, 3.0, -2.0));
    /// # Ok::<(), yack_base::Error>(())
    /// ```
    pub fn mirror(origin: impl Into<Point3>, normal: impl Into<Vec3>) -> Result<Similarity> {
        let n = Dir3::new(normal)?.vec().to_array();
        let o = origin.into().to_array();
        let d = 2.0 * (o[0] * n[0] + o[1] * n[1] + o[2] * n[2]);
        let row = |i: usize, ni: f64| {
            let r = |j: usize, nj: f64| if i == j { 1.0 - 2.0 * ni * nj } else { -2.0 * ni * nj };
            [r(0, n[0]), r(1, n[1]), r(2, n[2]), d * ni]
        };
        Similarity::from_rows([row(0, n[0]), row(1, n[1]), row(2, n[2])])
    }

    /// From rows `[c·Q | t]`: `Q` orthogonal (mirrors allowed), `c > 0`. Refuses anything else.
    ///
    /// ```
    /// use yack_math::*;
    /// // twice the size about the origin
    /// let m = Similarity::from_rows([[2.0, 0.0, 0.0, 0.0], [0.0, 2.0, 0.0, 0.0], [0.0, 0.0, 2.0, 0.0]])?;
    /// assert_eq!(m.point(Point3::new(1.0, 1.0, 1.0)), Point3::new(2.0, 2.0, 2.0));
    /// // a shear is not a similarity
    /// assert!(Similarity::from_rows([[1.0, 1.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0]]).is_err());
    /// # Ok::<(), yack_base::Error>(())
    /// ```
    pub fn from_rows(m: [[f64; 4]; 3]) -> Result<Similarity> {
        let op = "Similarity::from_rows";
        finite(op, &m)?;
        let scale = column(&m, 0).length();
        if scale <= yack_tol::ANGULAR {
            return Err(Error::invalid(op, "rows", "the scale is zero"));
        }
        orthogonal(op, &m, scale, yack_tol::ANGULAR)?;
        Ok(Similarity(m))
    }
    /// The rows `[c·Q | t]`.
    ///
    /// (Its example is on [`Similarity`].)
    pub const fn rows(&self) -> [[f64; 4]; 3] {
        self.0
    }
    /// The image of a point.
    ///
    /// ```
    /// use yack_math::*;
    /// let m = Similarity::from_rows([[1.0, 0.0, 0.0, 5.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0]])?;
    /// assert_eq!(m.point(Point3::ORIGIN), Point3::new(5.0, 0.0, 0.0));
    /// # Ok::<(), yack_base::Error>(())
    /// ```
    pub fn point(&self, p: Point3) -> Point3 {
        apply_point(&self.0, p)
    }
}

impl From<Rigid> for Similarity {
    fn from(r: Rigid) -> Similarity {
        Similarity(r.0)
    }
}

impl From<&Rigid> for Similarity {
    fn from(r: &Rigid) -> Similarity {
        Similarity(r.0)
    }
}

impl From<&Similarity> for Similarity {
    fn from(m: &Similarity) -> Similarity {
        *m
    }
}

impl Affine {
    /// The identity.
    pub const IDENTITY: Affine = Affine(IDENTITY);

    /// From rows `[A | t]`. Refuses a non-finite entry or a singular `A`.
    ///
    /// ```
    /// use yack_math::*;
    /// // a shear: invertible, so an affine map
    /// let m = Affine::from_rows([[1.0, 1.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0]])?;
    /// assert_eq!(m.point(Point3::new(0.0, 1.0, 0.0)), Point3::new(1.0, 1.0, 0.0));
    /// // a flattening one is refused
    /// assert!(Affine::from_rows([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]]).is_err());
    /// # Ok::<(), yack_base::Error>(())
    /// ```
    pub fn from_rows(m: [[f64; 4]; 3]) -> Result<Affine> {
        let op = "Affine::from_rows";
        finite(op, &m)?;
        let norms = column(&m, 0).length() * column(&m, 1).length() * column(&m, 2).length();
        let d = det(&m);
        if norms <= 0.0 || d.abs() <= yack_tol::ANGULAR * norms {
            return Err(Error::invalid(op, "rows", format!("the linear part is singular (det {d})")));
        }
        Ok(Affine(m))
    }
    /// The rows `[A | t]`.
    ///
    /// (Its example is on [`Affine`].)
    pub const fn rows(&self) -> [[f64; 4]; 3] {
        self.0
    }
    /// The image of a point.
    ///
    /// ```
    /// use yack_math::*;
    /// let m = Affine::from_rows([[2.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 1.0]])?;
    /// assert_eq!(m.point(Point3::new(1.0, 1.0, 1.0)), Point3::new(2.0, 1.0, 2.0));
    /// # Ok::<(), yack_base::Error>(())
    /// ```
    pub fn point(&self, p: Point3) -> Point3 {
        apply_point(&self.0, p)
    }
}

impl From<Similarity> for Affine {
    fn from(s: Similarity) -> Affine {
        Affine(s.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::deg;

    #[test]
    fn rotation_about_an_offset_axis() {
        let r = Rigid::rotation([1.0, 0.0, 0.0], [0.0, 0.0, 1.0], deg(90.0)).unwrap();
        let q = r.point(Point3::new(2.0, 0.0, 0.0));
        assert!(q.distance(Point3::new(1.0, 1.0, 0.0)) < yack_tol::LINEAR);
        assert!(Rigid::from_rows(r.rows()).is_ok());
    }

    #[test]
    fn mirror_is_a_similarity_not_a_rigid() {
        let m = [[-2.0, 0.0, 0.0, 0.0], [0.0, 2.0, 0.0, 0.0], [0.0, 0.0, 2.0, 0.0]];
        assert!(Rigid::from_rows(m).is_err());
        assert!(Similarity::from_rows(m).is_ok());
        assert!(Affine::from_rows(m).is_ok());
        assert!(Affine::from_rows([[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0]]).is_err());
    }
}
