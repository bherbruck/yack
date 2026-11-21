// yack: nalgebra-shaped rotations, quaternions, translations and isometries (f64).
//! The formulas are nalgebra's conventions (Rodrigues' matrix for `from_axis_angle`, the robust
//! trace-branching matrix-to-quaternion conversion, `q v q*` as `v + w t + q⃗ × t` with
//! `t = 2 q⃗ × v`), written out here so results agree with it.

use core::ops::{Index, Mul, Neg};

use super::matrix::Matrix;
use super::ops::matmul;
use super::point::Point;
use super::storage::{Scalar, Sh, Shape};
use super::unit::{Normed, Unit};

/// `relative_eq` with nalgebra's default epsilon, as nalgebra's `try_slerp` uses it.
const EPS: f64 = f64::EPSILON;

// ---- rotation matrices ----

/// A `D`-dimensional rotation matrix.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Rotation<T: Scalar, const D: usize>
where
    Sh: Shape<T, D, D>,
{
    matrix: Matrix<T, D, D>,
}

pub type Rotation2<T> = Rotation<T, 2>;
pub type Rotation3<T> = Rotation<T, 3>;

impl<const D: usize> Rotation<f64, D>
where
    Sh: Shape<f64, D, D>,
{
    /// No rotation.
    pub fn identity() -> Self {
        Self { matrix: Matrix::identity() }
    }
    /// `m`, taken to be orthonormal with determinant 1.
    pub fn from_matrix_unchecked(m: Matrix<f64, D, D>) -> Self {
        Self { matrix: m }
    }
    /// The matrix.
    pub fn matrix(&self) -> &Matrix<f64, D, D> {
        &self.matrix
    }
    /// The matrix.
    pub fn into_inner(self) -> Matrix<f64, D, D> {
        self.matrix
    }
    /// The inverse (the transpose).
    pub fn inverse(&self) -> Self {
        Self { matrix: self.matrix.transpose() }
    }
    /// The transpose (the inverse).
    pub fn transpose(&self) -> Self {
        self.inverse()
    }
    /// The homogeneous matrix.
    pub fn to_homogeneous(&self) -> Matrix<f64, 4, 4>
    where
        Sh: Shape<f64, 4, 4>,
    {
        Matrix::from_fn(|r, c| {
            if r < D && c < D {
                self.matrix[(r, c)]
            } else if r == c {
                1.0
            } else {
                0.0
            }
        })
    }
    /// `v` turned.
    pub fn transform_vector(&self, v: &Matrix<f64, D, 1>) -> Matrix<f64, D, 1>
    where
        Sh: Shape<f64, D, 1>,
    {
        matmul(&self.matrix, v)
    }
    /// `p` turned about the origin.
    pub fn transform_point(&self, p: &Point<f64, D>) -> Point<f64, D>
    where
        Sh: Shape<f64, D, 1>,
    {
        Point { coords: matmul(&self.matrix, &p.coords) }
    }
    /// `v` turned back.
    pub fn inverse_transform_vector(&self, v: &Matrix<f64, D, 1>) -> Matrix<f64, D, 1>
    where
        Sh: Shape<f64, D, 1>,
    {
        self.matrix.tr_mul(v)
    }
    /// `p` turned back.
    pub fn inverse_transform_point(&self, p: &Point<f64, D>) -> Point<f64, D>
    where
        Sh: Shape<f64, D, 1>,
    {
        Point { coords: self.matrix.tr_mul(&p.coords) }
    }
}

impl<const D: usize> Index<(usize, usize)> for Rotation<f64, D>
where
    Sh: Shape<f64, D, D>,
{
    type Output = f64;
    fn index(&self, i: (usize, usize)) -> &f64 {
        &self.matrix[i]
    }
}

impl Rotation<f64, 2> {
    /// The turn by `angle` radians.
    pub fn new(angle: f64) -> Self {
        let (s, c) = angle.sin_cos();
        Self { matrix: Matrix::<f64, 2, 2>::new(c, -s, s, c) }
    }
    /// The angle, in (−π, π].
    pub fn angle(&self) -> f64 {
        self.matrix[(1, 0)].atan2(self.matrix[(0, 0)])
    }
}

impl Rotation<f64, 3> {
    /// The turn by `angle` about `axis` (Rodrigues' matrix, as nalgebra builds it).
    pub fn from_axis_angle(axis: &Unit<Matrix<f64, 3, 1>>, angle: f64) -> Self {
        if angle == 0.0 {
            return Self::identity();
        }
        let (ux, uy, uz) = (axis.x, axis.y, axis.z);
        let sqx = ux * ux;
        let sqy = uy * uy;
        let sqz = uz * uz;
        let (sin, cos) = angle.sin_cos();
        let one_m_cos = 1.0 - cos;
        Self::from_matrix_unchecked(Matrix::<f64, 3, 3>::new(
            sqx + (1.0 - sqx) * cos,
            ux * uy * one_m_cos - uz * sin,
            ux * uz * one_m_cos + uy * sin,
            ux * uy * one_m_cos + uz * sin,
            sqy + (1.0 - sqy) * cos,
            uy * uz * one_m_cos - ux * sin,
            ux * uz * one_m_cos - uy * sin,
            uy * uz * one_m_cos + ux * sin,
            sqz + (1.0 - sqz) * cos,
        ))
    }
    /// The turn by `|v|` about `v`.
    pub fn from_scaled_axis(v: Matrix<f64, 3, 1>) -> Self {
        match Unit::try_new_and_get(v, 0.0) {
            Some((axis, angle)) => Self::from_axis_angle(&axis, angle),
            None => Self::identity(),
        }
    }
    /// `new(axisangle)`: [`Rotation3::from_scaled_axis`].
    pub fn new(axisangle: Matrix<f64, 3, 1>) -> Self {
        Self::from_scaled_axis(axisangle)
    }
    /// The smallest rotation taking `a`'s direction to `b`'s (`None` for opposite directions).
    pub fn rotation_between(a: &Matrix<f64, 3, 1>, b: &Matrix<f64, 3, 1>) -> Option<Self> {
        Self::scaled_rotation_between(a, b, 1.0)
    }
    /// [`Rotation3::rotation_between`] raised to the power `s`.
    pub fn scaled_rotation_between(
        a: &Matrix<f64, 3, 1>,
        b: &Matrix<f64, 3, 1>,
        s: f64,
    ) -> Option<Self> {
        if let (Some(na), Some(nb)) = (a.try_normalize(0.0), b.try_normalize(0.0)) {
            let c = na.cross(&nb);
            if let Some(axis) = Unit::try_new(c, EPS) {
                return Some(Self::from_axis_angle(&axis, na.dot(&nb).acos() * s));
            }
            if na.dot(&nb) < 0.0 {
                return None;
            }
        }
        Some(Self::identity())
    }
    /// The angle, in [0, π].
    pub fn angle(&self) -> f64 {
        ((self.matrix[(0, 0)] + self.matrix[(1, 1)] + self.matrix[(2, 2)] - 1.0) / 2.0).acos()
    }
    /// The axis (`None` for the identity, and for half turns).
    pub fn axis(&self) -> Option<Unit<Matrix<f64, 3, 1>>> {
        let m = &self.matrix;
        let axis = Matrix::<f64, 3, 1>::new(
            m[(2, 1)] - m[(1, 2)],
            m[(0, 2)] - m[(2, 0)],
            m[(1, 0)] - m[(0, 1)],
        );
        Unit::try_new(axis, EPS)
    }
    /// The axis times the angle.
    pub fn scaled_axis(&self) -> Matrix<f64, 3, 1> {
        match self.axis() {
            Some(axis) => axis.into_inner() * self.angle(),
            None => Matrix::zeros(),
        }
    }
    /// The axis and angle, unless there is no rotation.
    pub fn axis_angle(&self) -> Option<(Unit<Matrix<f64, 3, 1>>, f64)> {
        self.axis().map(|axis| (axis, self.angle()))
    }
}

macro_rules! rot_mul {
    ($($lt:lifetime)*; $A:ty, $B:ty, $Out:ty, |$a:ident, $b:ident| $e:expr) => {
        impl<$($lt,)* const D: usize> Mul<$B> for $A
        where
            Sh: Shape<f64, D, D> + Shape<f64, D, 1>,
        {
            type Output = $Out;
            #[allow(clippy::needless_borrow)]
            fn mul(self, $b: $B) -> $Out {
                let $a = &self;
                $e
            }
        }
    };
}
rot_mul!(; Rotation<f64, D>, Rotation<f64, D>, Rotation<f64, D>, |a, b| Rotation { matrix: matmul(&a.matrix, &b.matrix) });
rot_mul!('b; Rotation<f64, D>, &'b Rotation<f64, D>, Rotation<f64, D>, |a, b| Rotation { matrix: matmul(&a.matrix, &b.matrix) });
rot_mul!('a; &'a Rotation<f64, D>, Rotation<f64, D>, Rotation<f64, D>, |a, b| Rotation { matrix: matmul(&a.matrix, &b.matrix) });
rot_mul!('a 'b; &'a Rotation<f64, D>, &'b Rotation<f64, D>, Rotation<f64, D>, |a, b| Rotation { matrix: matmul(&a.matrix, &b.matrix) });
rot_mul!(; Rotation<f64, D>, Matrix<f64, D, 1>, Matrix<f64, D, 1>, |a, b| matmul(&a.matrix, &b));
rot_mul!('b; Rotation<f64, D>, &'b Matrix<f64, D, 1>, Matrix<f64, D, 1>, |a, b| matmul(&a.matrix, b));
rot_mul!('a; &'a Rotation<f64, D>, Matrix<f64, D, 1>, Matrix<f64, D, 1>, |a, b| matmul(&a.matrix, &b));
rot_mul!('a 'b; &'a Rotation<f64, D>, &'b Matrix<f64, D, 1>, Matrix<f64, D, 1>, |a, b| matmul(&a.matrix, b));
rot_mul!(; Rotation<f64, D>, Point<f64, D>, Point<f64, D>, |a, b| Point { coords: matmul(&a.matrix, &b.coords) });
rot_mul!('b; Rotation<f64, D>, &'b Point<f64, D>, Point<f64, D>, |a, b| Point { coords: matmul(&a.matrix, &b.coords) });
rot_mul!('a; &'a Rotation<f64, D>, Point<f64, D>, Point<f64, D>, |a, b| Point { coords: matmul(&a.matrix, &b.coords) });
rot_mul!('a 'b; &'a Rotation<f64, D>, &'b Point<f64, D>, Point<f64, D>, |a, b| Point { coords: matmul(&a.matrix, &b.coords) });
rot_mul!(; Rotation<f64, D>, Unit<Matrix<f64, D, 1>>, Unit<Matrix<f64, D, 1>>, |a, b| Unit::new_unchecked(matmul(&a.matrix, &b.value)));
rot_mul!('b; Rotation<f64, D>, &'b Unit<Matrix<f64, D, 1>>, Unit<Matrix<f64, D, 1>>, |a, b| Unit::new_unchecked(matmul(&a.matrix, &b.value)));
rot_mul!('a; &'a Rotation<f64, D>, Unit<Matrix<f64, D, 1>>, Unit<Matrix<f64, D, 1>>, |a, b| Unit::new_unchecked(matmul(&a.matrix, &b.value)));

// ---- quaternions ----

/// A quaternion `w + i·x + j·y + k·z`, stored `[x, y, z, w]` as nalgebra stores it.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Quaternion<T: Scalar>
where
    Sh: Shape<T, 4, 1>,
{
    /// `[i, j, k, w]`.
    pub coords: Matrix<T, 4, 1>,
}

/// A unit quaternion: a 3-D rotation.
pub type UnitQuaternion<T> = Unit<Quaternion<T>>;

impl Quaternion<f64> {
    /// `w + i x + j y + k z` (nalgebra's argument order: the scalar first).
    pub fn new(w: f64, i: f64, j: f64, k: f64) -> Self {
        Self { coords: Matrix::<f64, 4, 1>::new(i, j, k, w) }
    }
    /// The quaternion of scalar `w` and vector part `v`.
    pub fn from_parts(w: f64, v: Matrix<f64, 3, 1>) -> Self {
        Self::new(w, v.x, v.y, v.z)
    }
    /// The identity.
    pub fn identity() -> Self {
        Self::new(1.0, 0.0, 0.0, 0.0)
    }
    /// The vector part.
    pub fn vector(&self) -> Matrix<f64, 3, 1> {
        self.coords.xyz()
    }
    /// The scalar part.
    pub fn scalar(&self) -> f64 {
        self.coords.w
    }
    /// The scalar part.
    pub fn w(&self) -> f64 {
        self.coords.w
    }
    /// The conjugate.
    pub fn conjugate(&self) -> Self {
        Self::new(self.coords.w, -self.coords.x, -self.coords.y, -self.coords.z)
    }
    /// The 4-D dot product.
    pub fn dot(&self, b: &Self) -> f64 {
        self.coords.dot(&b.coords)
    }
    /// The norm.
    pub fn norm(&self) -> f64 {
        self.coords.norm()
    }
    /// The squared norm.
    pub fn norm_squared(&self) -> f64 {
        self.coords.norm_squared()
    }
    /// Divided by its norm.
    pub fn normalize(&self) -> Self {
        Self { coords: self.coords.normalize() }
    }
}

impl Normed for Quaternion<f64> {
    fn norm(&self) -> f64 {
        self.coords.norm()
    }
    fn norm_squared(&self) -> f64 {
        self.coords.norm_squared()
    }
    fn unscale(&self, n: f64) -> Self {
        Self { coords: self.coords.unscale(n) }
    }
}

impl Neg for Quaternion<f64> {
    type Output = Self;
    fn neg(self) -> Self {
        Self { coords: -self.coords }
    }
}

/// nalgebra's quaternion product, term for term.
fn qmul(a: &Quaternion<f64>, b: &Quaternion<f64>) -> Quaternion<f64> {
    let (a0, a1, a2, a3) = (a.coords[0], a.coords[1], a.coords[2], a.coords[3]);
    let (b0, b1, b2, b3) = (b.coords[0], b.coords[1], b.coords[2], b.coords[3]);
    Quaternion::new(
        a3 * b3 - a0 * b0 - a1 * b1 - a2 * b2,
        a3 * b0 + a0 * b3 + a1 * b2 - a2 * b1,
        a3 * b1 - a0 * b2 + a1 * b3 + a2 * b0,
        a3 * b2 + a0 * b1 - a1 * b0 + a2 * b3,
    )
}

impl Mul for Quaternion<f64> {
    type Output = Self;
    fn mul(self, b: Self) -> Self {
        qmul(&self, &b)
    }
}

impl Unit<Quaternion<f64>> {
    /// No rotation.
    pub fn identity() -> Self {
        Unit::new_unchecked(Quaternion::identity())
    }
    /// The turn by `angle` about `axis`.
    pub fn from_axis_angle(axis: &Unit<Matrix<f64, 3, 1>>, angle: f64) -> Self {
        let (sang, cang) = (angle / 2.0).sin_cos();
        Unit::new_unchecked(Quaternion::from_parts(cang, axis.value * sang))
    }
    /// The turn by `|v|` about `v`.
    pub fn from_scaled_axis(v: Matrix<f64, 3, 1>) -> Self {
        match Unit::try_new_and_get(v, 0.0) {
            Some((axis, angle)) => Self::from_axis_angle(&axis, angle),
            None => Self::identity(),
        }
    }
    /// `new(axisangle)`: [`UnitQuaternion::from_scaled_axis`].
    pub fn new(axisangle: Matrix<f64, 3, 1>) -> Self {
        Self::from_scaled_axis(axisangle)
    }
    /// `q` normalized.
    pub fn from_quaternion(q: Quaternion<f64>) -> Self {
        Unit::new_normalize(q)
    }
    /// The quaternion of rotation matrix `r` (nalgebra's trace-branching conversion).
    pub fn from_rotation_matrix(r: &Rotation<f64, 3>) -> Self {
        let m = |i: usize, j: usize| r.matrix[(i, j)];
        let tr = m(0, 0) + m(1, 1) + m(2, 2);
        let quarter = 0.25;
        let q = if tr > 0.0 {
            let denom = (tr + 1.0).sqrt() * 2.0;
            Quaternion::new(
                quarter * denom,
                (m(2, 1) - m(1, 2)) / denom,
                (m(0, 2) - m(2, 0)) / denom,
                (m(1, 0) - m(0, 1)) / denom,
            )
        } else if m(0, 0) > m(1, 1) && m(0, 0) > m(2, 2) {
            let denom = (1.0 + m(0, 0) - m(1, 1) - m(2, 2)).sqrt() * 2.0;
            Quaternion::new(
                (m(2, 1) - m(1, 2)) / denom,
                quarter * denom,
                (m(0, 1) + m(1, 0)) / denom,
                (m(0, 2) + m(2, 0)) / denom,
            )
        } else if m(1, 1) > m(2, 2) {
            let denom = (1.0 + m(1, 1) - m(0, 0) - m(2, 2)).sqrt() * 2.0;
            Quaternion::new(
                (m(0, 2) - m(2, 0)) / denom,
                (m(0, 1) + m(1, 0)) / denom,
                quarter * denom,
                (m(1, 2) + m(2, 1)) / denom,
            )
        } else {
            let denom = (1.0 + m(2, 2) - m(0, 0) - m(1, 1)).sqrt() * 2.0;
            Quaternion::new(
                (m(1, 0) - m(0, 1)) / denom,
                (m(0, 2) + m(2, 0)) / denom,
                (m(1, 2) + m(2, 1)) / denom,
                quarter * denom,
            )
        };
        Unit::new_unchecked(q)
    }
    /// The quaternion of `m`, taken to be a rotation matrix.
    pub fn from_matrix(m: &Matrix<f64, 3, 3>) -> Self {
        Self::from_rotation_matrix(&Rotation::from_matrix_unchecked(*m))
    }
    /// The quaternion.
    pub fn quaternion(&self) -> &Quaternion<f64> {
        &self.value
    }
    /// The scalar part.
    pub fn scalar(&self) -> f64 {
        self.value.coords.w
    }
    /// The rotation matrix, nalgebra's formula.
    pub fn to_rotation_matrix(self) -> Rotation<f64, 3> {
        let c = &self.value.coords;
        let (i, j, k, w) = (c[0], c[1], c[2], c[3]);
        let ww = w * w;
        let ii = i * i;
        let jj = j * j;
        let kk = k * k;
        let ij = i * j * 2.0;
        let wk = w * k * 2.0;
        let wj = w * j * 2.0;
        let ik = i * k * 2.0;
        let jk = j * k * 2.0;
        let wi = w * i * 2.0;
        Rotation::from_matrix_unchecked(Matrix::<f64, 3, 3>::new(
            ww + ii - jj - kk,
            ij - wk,
            wj + ik,
            wk + ij,
            ww - ii + jj - kk,
            jk - wi,
            ik - wj,
            wi + jk,
            ww - ii - jj + kk,
        ))
    }
    /// The homogeneous matrix.
    pub fn to_homogeneous(&self) -> Matrix<f64, 4, 4> {
        self.to_rotation_matrix().to_homogeneous()
    }
    /// The inverse (the conjugate).
    pub fn inverse(&self) -> Self {
        Unit::new_unchecked(self.value.conjugate())
    }
    /// The conjugate.
    pub fn conjugate(&self) -> Self {
        self.inverse()
    }
    /// The angle, in [0, π].
    pub fn angle(&self) -> f64 {
        let w = self.value.coords.w.abs();
        self.value.vector().norm().atan2(w) * 2.0
    }
    /// The axis, unless there is no rotation.
    pub fn axis(&self) -> Option<Unit<Matrix<f64, 3, 1>>> {
        let v = if self.value.coords.w >= 0.0 { self.value.vector() } else { -self.value.vector() };
        Unit::try_new(v, 0.0)
    }
    /// The axis times the angle.
    pub fn scaled_axis(&self) -> Matrix<f64, 3, 1> {
        match self.axis() {
            Some(axis) => axis.value * self.angle(),
            None => Matrix::zeros(),
        }
    }
    /// The axis and angle, unless there is no rotation.
    pub fn axis_angle(&self) -> Option<(Unit<Matrix<f64, 3, 1>>, f64)> {
        self.axis().map(|a| (a, self.angle()))
    }
    /// The angle of the rotation from `self` to `other`.
    pub fn angle_to(&self, other: &Self) -> f64 {
        (other * self.inverse()).angle()
    }
    /// The 4-D dot product.
    pub fn dot(&self, b: &Self) -> f64 {
        self.value.dot(&b.value)
    }
    /// Spherical interpolation along the shorter arc, nalgebra's formula. Where nalgebra would
    /// panic (the two all but equal yet not equal) this returns `self`.
    pub fn slerp(&self, other: &Self, t: f64) -> Self {
        self.try_slerp(other, t, EPS).unwrap_or(*self)
    }
    /// Spherical interpolation, `None` when the arc is ill-defined.
    pub fn try_slerp(&self, other: &Self, t: f64, epsilon: f64) -> Option<Self> {
        let a = self.value.coords;
        let b = if a.dot(&other.value.coords) < 0.0 {
            -other.value.coords
        } else {
            other.value.coords
        };
        let c_hang = a.dot(&b);
        if c_hang >= 1.0 {
            return Some(*self);
        }
        let hang = c_hang.acos();
        let s_hang = (1.0 - c_hang * c_hang).sqrt();
        if s_hang.abs() <= epsilon {
            None
        } else {
            let ta = ((1.0 - t) * hang).sin() / s_hang;
            let tb = (t * hang).sin() / s_hang;
            let res = Matrix::<f64, 4, 1>::from_fn(|i, _| tb * b[i] + a[i] * ta);
            Some(Unit::new_unchecked(Quaternion { coords: res }))
        }
    }
    /// Normalized linear interpolation.
    pub fn nlerp(&self, other: &Self, t: f64) -> Self {
        let mut res = self.value.coords.lerp(&other.value.coords, t);
        let _ = res.normalize_mut();
        Unit::new_unchecked(Quaternion { coords: res })
    }
    /// `v` turned, nalgebra's formula.
    pub fn transform_vector(&self, v: &Matrix<f64, 3, 1>) -> Matrix<f64, 3, 1> {
        let q = self.value.vector();
        let t = q.cross(v) * 2.0;
        let cross = q.cross(&t);
        t * self.value.coords.w + cross + v
    }
    /// `p` turned about the origin.
    pub fn transform_point(&self, p: &Point<f64, 3>) -> Point<f64, 3> {
        Point { coords: self.transform_vector(&p.coords) }
    }
    /// `v` turned back.
    pub fn inverse_transform_vector(&self, v: &Matrix<f64, 3, 1>) -> Matrix<f64, 3, 1> {
        self.inverse().transform_vector(v)
    }
    /// `p` turned back.
    pub fn inverse_transform_point(&self, p: &Point<f64, 3>) -> Point<f64, 3> {
        self.inverse().transform_point(p)
    }
    /// `v` turned back, kept unit.
    pub fn inverse_transform_unit_vector(
        &self,
        v: &Unit<Matrix<f64, 3, 1>>,
    ) -> Unit<Matrix<f64, 3, 1>> {
        Unit::new_unchecked(self.inverse_transform_vector(&v.value))
    }
    /// The rotation from `self` to `other`.
    pub fn rotation_to(&self, other: &Self) -> Self {
        other * self.inverse()
    }
    /// The smallest rotation taking `a`'s direction to `b`'s (`None` for opposite directions).
    pub fn rotation_between(a: &Matrix<f64, 3, 1>, b: &Matrix<f64, 3, 1>) -> Option<Self> {
        if let (Some(na), Some(nb)) = (a.try_normalize(0.0), b.try_normalize(0.0)) {
            let c = na.cross(&nb);
            if let Some(axis) = Unit::try_new(c, EPS) {
                return Some(Self::from_axis_angle(&axis, na.dot(&nb).acos()));
            }
            if na.dot(&nb) < 0.0 {
                return None;
            }
        }
        Some(Self::identity())
    }
}

macro_rules! uq_mul {
    ($($lt:lifetime)*; $B:ty, $Out:ty, |$a:ident, $b:ident| $e:expr) => {
        impl<$($lt)*> Mul<$B> for Unit<Quaternion<f64>> {
            type Output = $Out;
            #[allow(clippy::needless_borrow)]
            fn mul(self, $b: $B) -> $Out {
                let $a = &self;
                $e
            }
        }
        impl<'q, $($lt)*> Mul<$B> for &'q Unit<Quaternion<f64>> {
            type Output = $Out;
            #[allow(clippy::needless_borrow)]
            fn mul(self, $b: $B) -> $Out {
                let $a = self;
                $e
            }
        }
    };
}
uq_mul!(; Unit<Quaternion<f64>>, Unit<Quaternion<f64>>, |a, b| Unit::new_unchecked(qmul(&a.value, &b.value)));
uq_mul!('b; &'b Unit<Quaternion<f64>>, Unit<Quaternion<f64>>, |a, b| Unit::new_unchecked(qmul(&a.value, &b.value)));
uq_mul!(; Matrix<f64, 3, 1>, Matrix<f64, 3, 1>, |a, b| a.transform_vector(&b));
uq_mul!('b; &'b Matrix<f64, 3, 1>, Matrix<f64, 3, 1>, |a, b| a.transform_vector(b));
uq_mul!(; Point<f64, 3>, Point<f64, 3>, |a, b| a.transform_point(&b));
uq_mul!('b; &'b Point<f64, 3>, Point<f64, 3>, |a, b| a.transform_point(b));
uq_mul!(; Unit<Matrix<f64, 3, 1>>, Unit<Matrix<f64, 3, 1>>, |a, b| Unit::new_unchecked(a.transform_vector(&b.value)));
uq_mul!('b; &'b Unit<Matrix<f64, 3, 1>>, Unit<Matrix<f64, 3, 1>>, |a, b| Unit::new_unchecked(a.transform_vector(&b.value)));

impl From<Rotation<f64, 3>> for Unit<Quaternion<f64>> {
    fn from(r: Rotation<f64, 3>) -> Self {
        Self::from_rotation_matrix(&r)
    }
}
impl From<Unit<Quaternion<f64>>> for Rotation<f64, 3> {
    fn from(q: Unit<Quaternion<f64>>) -> Self {
        q.to_rotation_matrix()
    }
}
impl From<Unit<Quaternion<f64>>> for Matrix<f64, 3, 3> {
    fn from(q: Unit<Quaternion<f64>>) -> Self {
        q.to_rotation_matrix().into_inner()
    }
}

// ---- translations ----

/// A translation by `vector`.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Translation<T: Scalar, const D: usize>
where
    Sh: Shape<T, D, 1>,
{
    /// The displacement.
    pub vector: Matrix<T, D, 1>,
}

pub type Translation2<T> = Translation<T, 2>;
pub type Translation3<T> = Translation<T, 3>;

impl<const D: usize> Translation<f64, D>
where
    Sh: Shape<f64, D, 1>,
{
    /// No move.
    pub fn identity() -> Self {
        Self { vector: Matrix::zeros() }
    }
    /// The move back.
    pub fn inverse(&self) -> Self {
        Self { vector: -self.vector }
    }
    /// `p` moved.
    pub fn transform_point(&self, p: &Point<f64, D>) -> Point<f64, D> {
        Point { coords: p.coords + self.vector }
    }
    /// `p` moved back.
    pub fn inverse_transform_point(&self, p: &Point<f64, D>) -> Point<f64, D> {
        Point { coords: p.coords - self.vector }
    }
}

impl Translation<f64, 2> {
    pub fn new(x: f64, y: f64) -> Self {
        Self { vector: Matrix::<f64, 2, 1>::new(x, y) }
    }
}
impl Translation<f64, 3> {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { vector: Matrix::<f64, 3, 1>::new(x, y, z) }
    }
    /// The homogeneous matrix.
    pub fn to_homogeneous(&self) -> Matrix<f64, 4, 4> {
        Matrix::from_fn(|r, c| match (r, c) {
            (r, 3) if r < 3 => self.vector[r],
            (r, c) if r == c => 1.0,
            _ => 0.0,
        })
    }
}

impl<const D: usize> From<Matrix<f64, D, 1>> for Translation<f64, D>
where
    Sh: Shape<f64, D, 1>,
{
    fn from(vector: Matrix<f64, D, 1>) -> Self {
        Self { vector }
    }
}

impl<const D: usize> Mul<Point<f64, D>> for Translation<f64, D>
where
    Sh: Shape<f64, D, 1>,
{
    type Output = Point<f64, D>;
    fn mul(self, p: Point<f64, D>) -> Point<f64, D> {
        self.transform_point(&p)
    }
}
impl<'b, const D: usize> Mul<&'b Point<f64, D>> for Translation<f64, D>
where
    Sh: Shape<f64, D, 1>,
{
    type Output = Point<f64, D>;
    fn mul(self, p: &'b Point<f64, D>) -> Point<f64, D> {
        self.transform_point(p)
    }
}
impl Mul<Unit<Quaternion<f64>>> for Translation<f64, 3> {
    type Output = Isometry<f64, Unit<Quaternion<f64>>, 3>;
    fn mul(self, q: Unit<Quaternion<f64>>) -> Self::Output {
        Isometry::from_parts(self, q)
    }
}

// ---- isometries ----

/// A rotation `rotation` followed by the translation `translation`.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Isometry<T: Scalar, R, const D: usize>
where
    Sh: Shape<T, D, 1>,
{
    /// The rotation part.
    pub rotation: R,
    /// The translation part.
    pub translation: Translation<T, D>,
}

pub type Isometry3<T> = Isometry<T, Unit<Quaternion<T>>, 3>;
pub type IsometryMatrix3<T> = Isometry<T, Rotation<T, 3>, 3>;

type Iso3 = Isometry<f64, Unit<Quaternion<f64>>, 3>;

impl Iso3 {
    /// `rotation` then `translation`.
    pub fn from_parts(translation: Translation<f64, 3>, rotation: Unit<Quaternion<f64>>) -> Self {
        Self { rotation, translation }
    }
    /// No motion.
    pub fn identity() -> Self {
        Self::from_parts(Translation::identity(), Unit::<Quaternion<f64>>::identity())
    }
    /// The move by `(x, y, z)`.
    pub fn translation(x: f64, y: f64, z: f64) -> Self {
        Self::from_parts(Translation::<f64, 3>::new(x, y, z), Unit::<Quaternion<f64>>::identity())
    }
    /// The turn by `|axisangle|` about `axisangle` through the origin.
    pub fn rotation(axisangle: Matrix<f64, 3, 1>) -> Self {
        Self::from_parts(Translation::identity(), Unit::<Quaternion<f64>>::new(axisangle))
    }
    /// The turn by `|axisangle|` about `axisangle`, then the move by `translation`.
    pub fn new(translation: Matrix<f64, 3, 1>, axisangle: Matrix<f64, 3, 1>) -> Self {
        Self::from_parts(Translation::from(translation), Unit::<Quaternion<f64>>::new(axisangle))
    }
    /// The motion back.
    pub fn inverse(&self) -> Self {
        let rot = self.rotation.inverse();
        let v = rot.transform_vector(&self.translation.vector);
        Self::from_parts(Translation::from(-v), rot)
    }
    /// `p` moved.
    pub fn transform_point(&self, p: &Point<f64, 3>) -> Point<f64, 3> {
        Point { coords: self.rotation.transform_vector(&p.coords) + self.translation.vector }
    }
    /// `v` turned (translations don't move vectors).
    pub fn transform_vector(&self, v: &Matrix<f64, 3, 1>) -> Matrix<f64, 3, 1> {
        self.rotation.transform_vector(v)
    }
    /// `p` moved back.
    pub fn inverse_transform_point(&self, p: &Point<f64, 3>) -> Point<f64, 3> {
        self.rotation.inverse_transform_point(&Point { coords: p.coords - self.translation.vector })
    }
    /// `v` turned back.
    pub fn inverse_transform_vector(&self, v: &Matrix<f64, 3, 1>) -> Matrix<f64, 3, 1> {
        self.rotation.inverse_transform_vector(v)
    }
    /// The homogeneous matrix.
    pub fn to_homogeneous(&self) -> Matrix<f64, 4, 4> {
        let r = self.rotation.to_rotation_matrix();
        Matrix::from_fn(|i, j| match (i, j) {
            (i, 3) if i < 3 => self.translation.vector[i],
            (3, 3) => 1.0,
            (3, _) => 0.0,
            _ => r.matrix[(i, j)],
        })
    }
    /// The homogeneous matrix.
    pub fn to_matrix(&self) -> Matrix<f64, 4, 4> {
        self.to_homogeneous()
    }
    fn compose(&self, b: &Self) -> Self {
        let t = self.translation.vector + self.rotation.transform_vector(&b.translation.vector);
        Self::from_parts(Translation::from(t), self.rotation * b.rotation)
    }
}

macro_rules! iso_mul {
    ($($lt:lifetime)*; $B:ty, $Out:ty, |$a:ident, $b:ident| $e:expr) => {
        impl<$($lt)*> Mul<$B> for Iso3 {
            type Output = $Out;
            #[allow(clippy::needless_borrow)]
            fn mul(self, $b: $B) -> $Out {
                let $a = &self;
                $e
            }
        }
        impl<'q, $($lt)*> Mul<$B> for &'q Iso3 {
            type Output = $Out;
            #[allow(clippy::needless_borrow)]
            fn mul(self, $b: $B) -> $Out {
                let $a = self;
                $e
            }
        }
    };
}
iso_mul!(; Iso3, Iso3, |a, b| a.compose(&b));
iso_mul!('b; &'b Iso3, Iso3, |a, b| a.compose(b));
iso_mul!(; Point<f64, 3>, Point<f64, 3>, |a, b| a.transform_point(&b));
iso_mul!('b; &'b Point<f64, 3>, Point<f64, 3>, |a, b| a.transform_point(b));
iso_mul!(; Matrix<f64, 3, 1>, Matrix<f64, 3, 1>, |a, b| a.transform_vector(&b));
iso_mul!('b; &'b Matrix<f64, 3, 1>, Matrix<f64, 3, 1>, |a, b| a.transform_vector(b));
iso_mul!(; Unit<Matrix<f64, 3, 1>>, Unit<Matrix<f64, 3, 1>>, |a, b| Unit::new_unchecked(a.transform_vector(&b.value)));
iso_mul!('b; &'b Unit<Matrix<f64, 3, 1>>, Unit<Matrix<f64, 3, 1>>, |a, b| Unit::new_unchecked(a.transform_vector(&b.value)));
iso_mul!(; Unit<Quaternion<f64>>, Iso3, |a, b| Iso3::from_parts(a.translation, a.rotation * b));
iso_mul!(; Translation<f64, 3>, Iso3, |a, b| a.compose(&Iso3::from_parts(b, Unit::<Quaternion<f64>>::identity())));

impl From<Translation<f64, 3>> for Iso3 {
    fn from(t: Translation<f64, 3>) -> Self {
        Iso3::from_parts(t, Unit::<Quaternion<f64>>::identity())
    }
}

impl Default for Iso3 {
    fn default() -> Self {
        Iso3::identity()
    }
}
